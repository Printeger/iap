#!/usr/bin/env python3
# IAP-RQ-040 / IAP-RQ-002: ICP health + module timing plots — Fig C1, C2
# Usage: python3 tools/plot_icp_timing.py /tmp/iap_icp.csv /tmp/iap_timing.csv

import sys
import pathlib
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

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


# ---------------------------------------------------------------------------
# Fig C1: ICP health timeline (κ / inlier_fraction / γ_L)
# ---------------------------------------------------------------------------
def plot_C1(icp_df, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fid = icp_df["frame_id"].values

    # Panel 1: condition number κ
    ax = axes[0]
    ax.semilogy(fid, icp_df["condition_number"], "b-", linewidth=0.8, label="κ (cond number)")
    ax.axhline(500, color="r", linestyle="--", linewidth=0.7, label="threshold=500")
    ax.set_ylabel("Cond. Number κ")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which="both")
    ax.set_title("Fig C1 — LiDAR ICP Health Timeline")

    # Panel 2: inlier fraction
    ax = axes[1]
    ax.plot(fid, icp_df["inlier_fraction"] * 100.0, "g-", linewidth=0.8, label="Inlier fraction")
    ax.axhline(50, color="orange", linestyle="--", linewidth=0.7, label="50% guideline")
    ax.set_ylabel("Inlier Fraction [%]")
    ax.set_ylim(0, 105)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 3: γ_L (noise inflation factor)
    ax = axes[2]
    colors = ["#d62728" if v > 1.0 else "#2ca02c" for v in icp_df["gamma_lidar"]]
    ax.bar(fid, icp_df["gamma_lidar"], color=colors, alpha=0.75, width=1.0, label="γ_L")
    ax.axhline(1.0, color="k", linewidth=0.5)
    ax.set_ylabel("γ_L (noise inflation)")
    ax.set_xlabel("Frame ID")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")
    # Degenerate fraction
    n_degen = int((icp_df["drop_flag"] == 1).sum())
    ax.set_title(f"Degenerate frames: {n_degen}/{len(icp_df)} ({100*n_degen/max(len(icp_df),1):.1f}%)")

    fig.tight_layout()
    for ext in ("pdf", "png"):
        path = out_dir / f"fig_C1_icp_health.{ext}"
        fig.savefig(path, dpi=150)
        print(f"Saved {path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Fig C2: Module timing timeline + statistics table
# ---------------------------------------------------------------------------
def plot_C2(timing_df, icp_df, out_dir):
    modules = ["1.3_gnss_injection", "2.3_integrity_total", "2.1_gnss_araim", "1.3_trunk_detector",
               "2.2_lidar_araim", "4.0_plan_total", "3.0_pl_grid_build"]
    colors  = {"1.3_gnss_injection": "#1f77b4", "2.3_integrity_total": "#ff7f0e",
                "2.1_gnss_araim": "#2ca02c", "1.3_trunk_detector": "#9467bd",
                "2.2_lidar_araim": "#d62728", "4.0_plan_total": "#8c564b",
                "3.0_pl_grid_build": "#e377c2"}

    # Build per-module time series aligned to frame index
    fig = plt.figure(figsize=(14, 9))
    gs = GridSpec(2, 1, figure=fig, height_ratios=[3, 1])
    ax_plot  = fig.add_subplot(gs[0])
    ax_table = fig.add_subplot(gs[1])
    ax_table.axis("off")

    stats_rows = []

    for mod in modules:
        sub = timing_df[timing_df["module"] == mod].copy()
        if sub.empty:
            continue
        sub = sub.sort_values("stamp").reset_index(drop=True)
        t = sub["stamp"] - sub["stamp"].iloc[0]
        elapsed = sub["elapsed_ms"].values

        ax_plot.plot(t, elapsed, "-", color=colors[mod], linewidth=0.8,
                     label=mod, alpha=0.85)

        # Stats
        stats_rows.append([
            mod,
            f"{np.mean(elapsed):.2f}",
            f"{np.percentile(elapsed, 95):.2f}",
            f"{np.percentile(elapsed, 99):.2f}",
            f"{np.max(elapsed):.2f}",
        ])

    ax_plot.set_ylabel("Elapsed [ms]")
    ax_plot.set_xlabel("Time [s]")
    ax_plot.set_title("Fig C2 — Per-Module Timing (each frame)")
    ax_plot.legend(fontsize=8, loc="upper right")
    ax_plot.grid(True, alpha=0.3)

    # Stats table
    if stats_rows:
        col_labels = ["Module", "Mean [ms]", "p95 [ms]", "p99 [ms]", "Max [ms]"]
        tbl = ax_table.table(
            cellText=stats_rows,
            colLabels=col_labels,
            loc="center",
            cellLoc="center",
        )
        tbl.auto_set_font_size(True)
        tbl.scale(1, 1.5)
        # Colour p99 > 50 ms in red
        for row_i, row in enumerate(stats_rows):
            p99_val = float(row[3])
            if p99_val > 50.0:
                for col_j in range(len(col_labels)):
                    tbl[row_i + 1, col_j].set_facecolor("#ffcccc")

    fig.tight_layout()
    for ext in ("pdf", "png"):
        path = out_dir / f"fig_C2_module_timing.{ext}"
        fig.savefig(path, dpi=150)
        print(f"Saved {path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
def main():
    icp_path    = sys.argv[1] if len(sys.argv) > 1 else "/tmp/iap_icp.csv"
    timing_path = sys.argv[2] if len(sys.argv) > 2 else "/tmp/iap_timing.csv"
    out_dir     = pathlib.Path(sys.argv[3]) if len(sys.argv) > 3 else OUT_DIR

    out_dir.mkdir(parents=True, exist_ok=True)

    icp_path_resolved = resolve_input_path(icp_path)
    timing_path_resolved = resolve_input_path(timing_path)

    print(f"Loading ICP CSV   : {icp_path_resolved}")
    icp_df = pd.read_csv(icp_path_resolved)
    print(f"  {len(icp_df)} ICP frames")

    print(f"Loading timing CSV: {timing_path_resolved}")
    timing_df = pd.read_csv(timing_path_resolved)
    print(f"  {len(timing_df)} timing rows, modules: {timing_df['module'].unique().tolist()}")

    plot_C1(icp_df, out_dir)
    plot_C2(timing_df, icp_df, out_dir)
    print("Done — Figs C1/C2 saved.")


if __name__ == "__main__":
    main()
