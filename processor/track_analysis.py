import json
import os

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

A_LAT = float(os.environ.get("A_LAT", 12.0))      # max lateral acceleration [m/s^2]
V_TOP = float(os.environ.get("V_TOP", 30.0))      # top speed cap [m/s]
SMOOTHING = 2.0                                   # spline smoothing [m]
N_SAMPLES = 2000                                  # points on the smooth path


def analyze(left, right, out_dir):
    """left, right: Nx2 arrays. Saves plots and results.json into out_dir."""
    # Midline and track width
    mid = (left + right) / 2
    width = np.hypot(*(left - right).T)

    # Close the loop and smooth with a periodic spline
    mid_closed = np.vstack([mid, mid[0]])
    tck, _ = splprep(mid_closed.T, s=len(mid) * SMOOTHING**2, per=True)
    u = np.linspace(0, 1, N_SAMPLES, endpoint=False)
    x, y = splev(u, tck)
    dx, dy = splev(u, tck, der=1)
    ddx, ddy = splev(u, tck, der=2)

    # Curvature
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5

    # Distance along the path
    px, py = np.append(x, x[0]), np.append(y, y[0])
    ds = np.hypot(np.diff(px), np.diff(py))
    s = np.concatenate([[0], np.cumsum(ds)[:-1]])
    length = ds.sum()

    # Max speed and lap time
    v_max = np.minimum(V_TOP, np.sqrt(A_LAT / np.maximum(np.abs(curvature), 1e-9)))
    v_next = np.roll(v_max, -1)
    lap_time = np.sum(ds / ((v_max + v_next) / 2))

    # Plots
    os.makedirs(out_dir, exist_ok=True)
    plt.figure(figsize=(8, 8))
    plt.plot(left[:, 0], left[:, 1], "k-", lw=0.8)
    plt.plot(right[:, 0], right[:, 1], "k-", lw=0.8)
    sc = plt.scatter(x, y, c=v_max * 3.6, cmap="RdYlGn", s=6)
    plt.colorbar(sc, label="max speed [km/h]")
    plt.axis("equal")
    plt.title("Brands Hatch - speed along the centerline")
    plt.savefig(os.path.join(out_dir, "track_speed.png"), dpi=150)
    plt.close()

    plt.figure(figsize=(10, 4))
    plt.plot(s, v_max * 3.6)
    plt.xlabel("distance [m]")
    plt.ylabel("max speed [km/h]")
    plt.title("Speed profile")
    plt.grid(True)
    plt.savefig(os.path.join(out_dir, "speed_profile.png"), dpi=150)
    plt.close()

    results = {
        "path_length_m": round(float(length), 1),
        "lap_time_s": round(float(lap_time), 1),
        "max_speed_kmh": round(float(v_max.max() * 3.6), 1),
        "min_speed_kmh": round(float(v_max.min() * 3.6), 1),
        "max_abs_curvature_1_per_m": round(float(np.abs(curvature).max()), 4),
        "mean_track_width_m": round(float(width.mean()), 1),
        "a_lat": A_LAT,
        "v_top": V_TOP,
    }
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(results, f, indent=2)
    return results