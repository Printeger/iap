#!/usr/bin/env python3
# IAP-RQ-002: Trajectory comparison — Fig D1 (LIO-only vs LIO+GNSS)
# Usage: python3 tools/plot_trajectory_comparison.py \
#            /tmp/traj_with_gnss.csv /tmp/traj_without_gnss.csv

import sys
import pathlib
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT_DIR = pathlib.Path("/home/dev/code/ws_iap/src/iap/tools/figs")


def resolve_input_path(path_str: str) -> pathlib.Path:
    path = pathlib.Path(path_str)
    if not path.exists() and not path.is_absolute():
        alt = pathlib.Path("/tmp") / path
        if alt.exists():
            path = alt
    if not path.exists():
        raise FileNotFoundError(
            f"Input CSV not found: {path_str} (also tried: /tmp/{pathlib.Path(path_str).name})"
        )
    return path


def load_traj(path):
    df = pd.read_csv(resolve_input_path(path))
    return df["x"].values, df["y"].values, df["z"].values, df["stamp"].values


def endpoint_drift(x, y):
    """Distance from first to last point [m]."""
    return float(np.hypot(x[-1] - x[0], y[-1] - y[0]))


def plot_D1(traj_gnss_path, traj_no_gnss_path, out_dir):
    x_g, y_g, z_g, t_g = load_traj(traj_gnss_path)
    x_n, y_n, z_n, t_n = load_traj(traj_no_gnss_path)

    drift_gnss    = endpoint_drift(x_g, y_g)
    drift_no_gnss = endpoint_drift(x_n, y_n)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # ---- Panel 1: 2D overhead trajectories ----
    ax = axes[0]
    ax.plot(x_n, y_n, "r-",  linewidth=0.9, label=f"LIO-only   (Δxy={drift_no_gnss:.2f} m)")
    ax.plot(x_g, y_g, "b-",  linewidth=0.9, label=f"LIO+GNSS   (Δxy={drift_gnss:.2f} m)")

    # Mark start/end
    ax.scatter([x_g[0], x_n[0]], [y_g[0], y_n[0]], color="green", zorder=5,
               s=60, marker="o", label="Start")
    ax.scatter([x_g[-1]], [y_g[-1]], color="blue", zorder=5, s=60, marker="X")
    ax.scatter([x_n[-1]], [y_n[-1]], color="red",  zorder=5, s=60, marker="X")

    # Annotate end-point drift delta
    ax.annotate(f"Δxy = {abs(drift_gnss - drift_no_gnss):.2f} m",
                xy=(x_g[-1], y_g[-1]),
                xytext=(x_g[-1] + 1.5, y_g[-1] + 1.5),
                fontsize=9,
                arrowprops=dict(arrowstyle="->", color="k", lw=0.8),
                bbox=dict(boxstyle="round,pad=0.2", fc="lightyellow", alpha=0.8))

    ax.set_xlabel("X (East) [m]")
    ax.set_ylabel("Y (North) [m]")
    ax.set_aspect("equal")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_title("Fig D1 — 2D Trajectory Comparison (top view)")

    # ---- Panel 2: Cumulative drift over time ----
    ax2 = axes[1]

    def cumulative_drift(x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        steps = np.sqrt(dx**2 + dy**2)
        dist  = np.concatenate([[0], np.cumsum(steps)])
        # "drift" = distance to start point
        return np.sqrt((x - x[0])**2 + (y - y[0])**2)

    drift_g = cumulative_drift(x_g, y_g)
    drift_n = cumulative_drift(x_n, y_n)

    t_g_rel = t_g - t_g[0]
    t_n_rel = t_n - t_n[0]

    ax2.plot(t_n_rel, drift_n, "r-", linewidth=0.9, label="LIO-only")
    ax2.plot(t_g_rel, drift_g, "b-", linewidth=0.9, label="LIO+GNSS")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Distance to origin [m]")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    ax2.set_title("Trajectory spread over time")

    # Metrics text box
    textstr = (f"LIO-only endpoint drift: {drift_no_gnss:.2f} m\n"
               f"LIO+GNSS endpoint drift: {drift_gnss:.2f} m\n"
               f"Improvement: {max(drift_no_gnss - drift_gnss, 0):.2f} m")
    ax2.text(0.97, 0.05, textstr, transform=ax2.transAxes,
             fontsize=9, verticalalignment="bottom", horizontalalignment="right",
             bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.8))

    fig.tight_layout()
    for ext in ("pdf", "png"):
        path = out_dir / f"fig_D1_trajectory_comparison.{ext}"
        fig.savefig(path, dpi=150)
        print(f"Saved {path}")
    plt.close(fig)


def main():
    gnss_path    = sys.argv[1] if len(sys.argv) > 1 else "/tmp/traj_with_gnss.csv"
    no_gnss_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/traj_without_gnss.csv"
    out_dir      = pathlib.Path(sys.argv[3]) if len(sys.argv) > 3 else OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"LIO+GNSS   trajectory: {gnss_path}")
    print(f"LIO-only   trajectory: {no_gnss_path}")

    plot_D1(gnss_path, no_gnss_path, out_dir)
    print("Done — Fig D1 saved.")


if __name__ == "__main__":
    main()
