#!/usr/bin/env python3
import csv
import math
import os
import random


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "results", "araim_validation")
RNG = random.Random(20260603)


def write_csv(name, points):
    path = os.path.join(OUT, name)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        for point in points:
            writer.writerow(point)
    print("wrote", path, len(points), "points")


def jitter(value, scale=0.015):
    return value + RNG.uniform(-scale, scale)


def feature_rich():
    points = []
    for _ in range(900):
        y = RNG.uniform(-5.0, 5.0)
        z = RNG.uniform(0.0, 4.0)
        points.append((jitter(5.0), y, z))
        points.append((jitter(-5.0), y, z))

        x = RNG.uniform(-5.0, 5.0)
        points.append((x, jitter(5.0), z))
        points.append((x, jitter(-5.0), z))

        points.append((x, y, jitter(0.0)))

    for cx, cy in [(-2.5, -2.5), (-2.5, 2.5), (2.5, -2.5), (2.5, 2.5)]:
        for _ in range(220):
            theta = RNG.uniform(0.0, 2.0 * 3.141592653589793)
            radius = 0.35 + RNG.uniform(-0.01, 0.01)
            z = RNG.uniform(0.0, 4.0)
            points.append((cx + radius * math.cos(theta),
                           cy + radius * math.sin(theta),
                           z))
    return points


def corridor():
    points = []
    for _ in range(1200):
        x = RNG.uniform(-10.0, 10.0)
        z = RNG.uniform(0.0, 3.0)
        points.append((x, jitter(-2.0), z))
        points.append((x, jitter(2.0), z))
        points.append((x, RNG.uniform(-2.0, 2.0), jitter(0.0)))
    return points


def sparse():
    points = []
    for _ in range(80):
        x = RNG.uniform(-5.0, 5.0)
        z = RNG.uniform(0.0, 3.0)
        points.append((x, jitter(2.0), z))
    return points


AXIS_LIMITS = {
    "x": (-10.5, 10.5),
    "y": (-5.5, 5.5),
    "z": (-0.2, 4.2),
}


def style_cloud_axis(ax, title):
    ax.set_title(title, fontsize=11)
    ax.set_xlabel("x [m]", labelpad=5)
    ax.set_ylabel("y [m]", labelpad=5)
    ax.set_zlabel("z [m]", labelpad=5)
    ax.set_xlim(*AXIS_LIMITS["x"])
    ax.set_ylim(*AXIS_LIMITS["y"])
    ax.set_zlim(*AXIS_LIMITS["z"])
    ax.set_xticks([-10, -5, 0, 5, 10])
    ax.set_yticks([-5, 0, 5])
    ax.set_zticks([0, 2, 4])
    ax.view_init(elev=24, azim=-58)
    ax.set_box_aspect((
        AXIS_LIMITS["x"][1] - AXIS_LIMITS["x"][0],
        AXIS_LIMITS["y"][1] - AXIS_LIMITS["y"][0],
        AXIS_LIMITS["z"][1] - AXIS_LIMITS["z"][0],
    ))
    ax.tick_params(axis="both", which="major", labelsize=8, pad=1)


def write_plot(name, points, title=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("matplotlib unavailable; skipped", name, ":", exc)
        return

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(111, projection="3d")
    scatter = ax.scatter(xs, ys, zs, c=zs, cmap="viridis",
                         vmin=0.0, vmax=4.0, s=1.2, alpha=0.55)
    style_cloud_axis(ax, title or name.replace("_", " ").replace(".png", ""))
    fig.colorbar(scatter, ax=ax, shrink=0.65, pad=0.08, label="z [m]")
    path = os.path.join(OUT, name)
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    print("wrote", path)


def write_combined_plot(clouds):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("matplotlib unavailable; skipped lidar_cloud_inputs_comparison.png :", exc)
        return

    titles = [
        ("feature_rich_cloud.csv", "(a) Feature-rich"),
        ("corridor_cloud.csv", "(b) Corridor"),
        ("sparse_cloud.csv", "(c) Sparse"),
    ]
    fig = plt.figure(figsize=(13.5, 4.4))
    scatter = None
    for index, (name, title) in enumerate(titles, start=1):
        points = clouds[name]
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        zs = [p[2] for p in points]
        ax = fig.add_subplot(1, 3, index, projection="3d")
        scatter = ax.scatter(xs, ys, zs, c=zs, cmap="viridis",
                             vmin=0.0, vmax=4.0, s=1.0, alpha=0.55)
        style_cloud_axis(ax, title)

    if scatter is not None:
        cbar = fig.colorbar(scatter, ax=fig.axes, shrink=0.74,
                            pad=0.03, fraction=0.03)
        cbar.set_label("height z [m]")
    path = os.path.join(OUT, "lidar_cloud_inputs_comparison.png")
    fig.savefig(path, dpi=220, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


def main():
    os.makedirs(OUT, exist_ok=True)
    clouds = {
        "feature_rich_cloud.csv": feature_rich(),
        "corridor_cloud.csv": corridor(),
        "sparse_cloud.csv": sparse(),
    }
    for name, points in clouds.items():
        write_csv(name, points)
        write_plot(name.replace(".csv", ".png"), points,
                   name.replace("_cloud.csv", "").replace("_", " ").title())
    write_combined_plot(clouds)


if __name__ == "__main__":
    main()
