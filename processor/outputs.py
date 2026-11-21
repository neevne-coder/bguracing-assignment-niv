import json
import os

import numpy as np
import matplotlib.pyplot as plt


def save_outputs(out_dir, left, right, corridor, offsets, line, speed, distance,
                 throttle, brake, results):
    os.makedirs(out_dir, exist_ok=True)
    along = np.concatenate([[0], np.cumsum(distance)[:-1]])  # distance from the start

    # Whole track, racing line coloured by speed
    plt.figure(figsize=(8, 8))
    draw_track(plt.gca(), left, right, corridor)
    sc = plt.scatter(line[:, 0], line[:, 1], c=speed * 3.6, cmap="RdYlGn", s=6, zorder=3)
    plt.colorbar(sc, label="speed [km/h]")
    plt.title("Racing line (colour = speed)")
    plt.savefig(os.path.join(out_dir, "racing_line.png"), dpi=150)
    plt.close()

    # Zoom into the three sharpest corners
    corners = sharpest_corners(corridor["center"], 3)
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    for ax, i in zip(axes, corners):
        draw_track(ax, left, right, corridor)
        ax.scatter(line[:, 0], line[:, 1], c=speed * 3.6, cmap="RdYlGn", s=14, zorder=3)
        cx, cy = corridor["center"][i]
        ax.set_xlim(cx - 70, cx + 70)
        ax.set_ylim(cy - 70, cy + 70)
        ax.set_title(f"Corner at {along[i]:.0f} m")
    plt.savefig(os.path.join(out_dir, "corners_zoom.png"), dpi=130)
    plt.close()

    # Speed, offset from the center, throttle and brake along the lap
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(along, speed * 3.6)
    ax[0].set_ylabel("speed [km/h]")
    ax[1].plot(along, offsets)
    ax[1].axhline(0, color="gray", lw=0.5)
    ax[1].set_ylabel("offset from center [m]\n(+ = left)")
    ax[2].plot(along, throttle, label="throttle")
    ax[2].plot(along, brake, label="brake")
    ax[2].set_ylabel("pedals [%]")
    ax[2].set_xlabel("distance [m]")
    ax[2].legend()
    plt.savefig(os.path.join(out_dir, "profiles.png"), dpi=130)
    plt.close()

    # Table with everything, one row per point
    table = np.column_stack([along, line, offsets, speed * 3.6, throttle, brake])
    np.savetxt(os.path.join(out_dir, "racing_line.csv"), table, delimiter=",", fmt="%.2f",
               header="distance_m,x,y,offset_m,speed_kmh,throttle_pct,brake_pct", comments="")

    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(results, f, indent=2)


def draw_track(ax, left, right, corridor):
    ax.plot(left[:, 0], left[:, 1], "k", lw=0.8)
    ax.plot(right[:, 0], right[:, 1], "k", lw=0.8)
    ax.plot(*np.append(corridor["center"], corridor["center"][:1], axis=0).T, "k--", lw=0.5)
    ax.set_aspect("equal")


def sharpest_corners(center, count, min_gap=40):
    """Indices of the sharpest turns, at least min_gap points apart."""
    from racing_line import curvature
    sharp = np.abs(curvature(center))
    chosen = []
    for i in np.argsort(sharp)[::-1]:
        if all(min(abs(i - j), len(center) - abs(i - j)) > min_gap for j in chosen):
            chosen.append(i)
        if len(chosen) == count:
            break
    return sorted(chosen)