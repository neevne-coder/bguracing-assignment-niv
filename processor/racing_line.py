import numpy as np
from scipy.spatial import cKDTree

G = 9.81  # gravity, m/s^2
G = 9.81  # gravity, m/s^2

# Grip (friction coefficient) of every tire on a dry and on a wet surface (ASSUMED values)
TIRE_GRIP = {
    "soft":   {"dry": 1.70, "wet": 0.85},
    "medium": {"dry": 1.60, "wet": 0.80},
    "hard":   {"dry": 1.50, "wet": 0.70},
    "wet":    {"dry": 1.00, "wet": 1.10},
}


def resample(points, n):
    """Take n evenly spaced points from a closed polyline. Spacing is by point index,
    so the left and right boundary keep matching each other."""
    index = np.arange(len(points) + 1)
    closed = np.vstack([points, points[0]])
    new_index = np.linspace(0, len(points), n, endpoint=False)
    return np.column_stack([np.interp(new_index, index, closed[:, 0]),
                            np.interp(new_index, index, closed[:, 1])])


def smooth(points, passes):
    """Round off corners: replace every point by the average of itself and its neighbours."""
    for _ in range(passes):
        points = (np.roll(points, 1, axis=0) + points + np.roll(points, -1, axis=0)) / 3
    return points


def make_corridor(left, right, margin, n=400):
    """Describe the track as a corridor: smooth center line, direction to the left, and how far
    we may move to the left / right from the center while staying `margin` meters from the walls."""
    center = smooth((resample(left, n) + resample(right, n)) / 2, 8)

    # Direction to the left = the direction of travel turned by 90 degrees
    tangent = np.roll(center, -1, axis=0) - np.roll(center, 1, axis=0)
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    normal /= np.hypot(*normal.T)[:, None]
    if np.mean(np.sum((resample(left, n) - center) * normal, axis=1)) < 0:
        normal = -normal

    # The track walls as many close points, so we can ask "how far is the nearest wall?"
    walls = cKDTree(np.vstack([resample(left, 8000), resample(right, 8000)]))

    # From every center point, take small steps to each side and stop right before
    # we get closer than `margin` to a wall
    steps = np.arange(0, 40, 0.25)
    limits = {}
    for side, sign in (("left", 1), ("right", -1)):
        spots = center[:, None, :] + sign * steps[None, :, None] * normal[:, None, :]
        too_close = walls.query(spots)[0] < margin                   # distance to nearest wall
        first_bad = np.where(too_close.any(axis=1), too_close.argmax(axis=1), len(steps))
        limits[side] = steps[np.maximum(first_bad - 1, 0)]
    return {"center": center, "normal": normal, "walls": walls,
            "limit_left": limits["left"], "limit_right": limits["right"]}


def make_line(corridor, offsets):
    """Racing line: move each center point sideways by its offset (meters, + = left)."""
    return corridor["center"] + offsets[:, None] * corridor["normal"]


def curvature(points):
    """How sharp the turn is at each point (1 / radius of the circle through 3 neighbours)."""
    a, b, c = np.roll(points, 1, axis=0), points, np.roll(points, -1, axis=0)
    ab = np.hypot(*(b - a).T)
    bc = np.hypot(*(c - b).T)
    ca = np.hypot(*(a - c).T)
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    return 2 * cross / (ab * bc * ca)


def speed_profile(points, car, grip):
    """Fastest possible speed at each point and the lap time.
    Returns speed (m/s), distance to the next point (m) and lap time (s)."""
    distance = np.hypot(*(np.roll(points, -1, axis=0) - points).T)
    top_speed = car["max_speed_kmh"] / 3.6

    # 1. Corner limit: the sharper the turn, the slower. v = sqrt(grip * g / curvature)
    speed = np.minimum(top_speed, np.sqrt(grip * G / np.maximum(np.abs(curvature(points)), 1e-9)))

    # Go around the lap 3 times so the end of the lap connects to the start
    n = len(speed)
    speed3, dist3 = np.tile(speed, 3), np.tile(distance, 3)

    # 2. Acceleration limit: speed can only grow as fast as the car accelerates
    for i in range(len(speed3) - 1):
        reachable = np.sqrt(speed3[i] ** 2 + 2 * car["max_accel"] * dist3[i])
        speed3[i + 1] = min(speed3[i + 1], reachable)

    # 3. Braking limit: looking backwards, speed must allow braking in time for the next point
    for i in range(len(speed3) - 2, -1, -1):
        allowed = np.sqrt(speed3[i + 1] ** 2 + 2 * car["max_brake"] * dist3[i])
        speed3[i] = min(speed3[i], allowed)

    speed = speed3[n:2 * n]  # keep the middle lap
    next_speed = np.roll(speed, -1)
    lap_time = float(np.sum(distance / ((speed + next_speed) / 2)))
    return speed, distance, lap_time


def pedal_commands(speed, distance, car):
    """Throttle and brake (0-100 %) needed to follow the speed profile."""
    accel = (np.roll(speed, -1) ** 2 - speed ** 2) / (2 * distance)  # m/s^2 along the line
    throttle = np.clip(accel / car["max_accel"], 0, 1) * 100
    brake = np.clip(-accel / car["max_brake"], 0, 1) * 100
    return throttle, brake


def offsets_from_controls(controls, corridor):
    """Spread a few control values smoothly over all points and keep them inside the safe corridor."""
    n = len(corridor["center"])
    positions = np.linspace(0, n, len(controls), endpoint=False)
    offsets = np.interp(np.arange(n), np.append(positions, n), np.append(controls, controls[0]))
    offsets = smooth(offsets[:, None], 3)[:, 0]
    return np.clip(offsets, -corridor["limit_right"], corridor["limit_left"])


def optimize_line(corridor, car, grip, iterations=10000, n_controls=60, seed=0):
    """Find the offsets with the shortest lap time by trial and error (hill climbing):
    change some offsets randomly and keep the change only if the lap gets faster."""
    n = len(corridor["center"])
    rng = np.random.default_rng(seed)
    at_controls = np.linspace(0, n, n_controls, endpoint=False).astype(int)
    lowest = -corridor["limit_right"][at_controls]
    highest = corridor["limit_left"][at_controls]

    def lap_time(controls):
        offsets = offsets_from_controls(controls, corridor)
        return speed_profile(make_line(corridor, offsets), car, grip)[2]

    best = np.zeros(n_controls)         # start on the center line
    best_time = lap_time(best)
    for i in range(iterations):
        step = 4.0 * (1 - i / iterations) + 0.3                  # big steps first, small later
        change = step * rng.standard_normal(n_controls) * (rng.random(n_controls) < 0.2)
        candidate = np.clip(best + change, lowest, highest)
        candidate_time = lap_time(candidate)
        if candidate_time < best_time:
            best, best_time = candidate, candidate_time
    return offsets_from_controls(best, corridor)