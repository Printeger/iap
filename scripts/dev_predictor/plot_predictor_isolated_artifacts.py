#!/usr/bin/env python3
"""Plot Predictor isolated-test artifacts for the mentor-facing report."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


COLORS = {
    "gnss": "#2f6f73",
    "hpl": "#6c5b7b",
    "vpl": "#b56576",
    "lidar": "#6a994e",
    "warn": "#b08900",
    "bad": "#b34040",
    "muted": "#777777",
    "pred": "#3d405b",
}


def parse_value(value: str):
    if value is None:
        return value
    text = value.strip()
    if text == "":
        return text
    try:
        return float(text)
    except ValueError:
        return text


def read_csv(path: Path) -> list[dict[str, object]]:
    with path.open(newline="") as f:
      return [{key: parse_value(value) for key, value in row.items()}
              for row in csv.DictReader(f)]


def finite(value: object) -> bool:
    return isinstance(value, float) and math.isfinite(value)


def save(fig: plt.Figure, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_gnss_geometry_sweep(artifact_dir: Path) -> None:
    rows = read_csv(artifact_dir / "gnss_geometry_sweep.csv")
    labels = [str(row["case_id"]) for row in rows]
    x = list(range(len(labels)))
    pdop = [row["pdop"] if finite(row["pdop"]) else math.nan for row in rows]
    hpl = [row["hpl"] if finite(row["hpl"]) else math.nan for row in rows]
    vpl = [row["vpl"] if finite(row["vpl"]) else math.nan for row in rows]

    fig, ax1 = plt.subplots(figsize=(10.5, 5.2))
    ax2 = ax1.twinx()
    ax1.plot(x, pdop, marker="o", color=COLORS["gnss"], label="PDOP")
    ax2.plot(x, hpl, marker="s", color=COLORS["hpl"], label="HPL")
    ax2.plot(x, vpl, marker="^", color=COLORS["vpl"], label="VPL")
    ax2.set_yscale("log")
    ax1.set_ylabel("PDOP")
    ax2.set_ylabel("Protection level (log scale)")
    ax1.set_xticks(x, labels, rotation=25, ha="right")
    ax1.set_title("GNSS geometry sweep: geometry degradation response")
    ax1.grid(axis="y", alpha=0.25)
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, frameon=False, loc="upper left")
    save(fig, artifact_dir / "gnss_geometry_sweep_pdop_pl.png")


def plot_gnss_geometry_skyplots(artifact_dir: Path) -> None:
    cases = [
        ("uniform_high", [0, 45, 90, 135, 180, 225, 270, 315],
         [25.8, 30.4, 35.0, 39.5, 25.8, 30.4, 35.0, 39.5]),
        ("single_quadrant", [0, 15, 30, 45, 60, 75, 90, 105],
         [50, 48, 46, 44, 42, 40, 38, 36]),
        ("low_elevation", [0, 45, 90, 135, 180, 225, 270, 315],
         [14, 15, 13, 16, 14, 15, 13, 16]),
    ]
    fig = plt.figure(figsize=(12, 4.2))
    for idx, (name, az_deg, el_deg) in enumerate(cases, start=1):
        ax = fig.add_subplot(1, 3, idx, projection="polar")
        az = [math.radians(v) for v in az_deg]
        r = [90.0 - v for v in el_deg]
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_rlim(90, 0)
        ax.set_rticks([0, 30, 60, 90])
        ax.set_yticklabels(["90", "60", "30", "0"])
        ax.scatter(az, r, s=70, c=COLORS["gnss"], edgecolors="white", zorder=3)
        ax.set_title(name)
    fig.suptitle("Representative GNSS sweep skyplots")
    save(fig, artifact_dir / "gnss_geometry_sweep_skyplots.png")


def plot_gnss_sigma_sweep(artifact_dir: Path) -> None:
    rows = read_csv(artifact_dir / "gnss_sigma_sweep.csv")
    x = [row["sigma_scale"] for row in rows]
    hpl = [row["hpl"] for row in rows]
    vpl = [row["vpl"] for row in rows]
    trace = [row["lambda_gnss_trace"] for row in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.5, 4.6))
    ax1.plot(x, hpl, marker="o", label="HPL", color=COLORS["hpl"])
    ax1.plot(x, vpl, marker="s", label="VPL", color=COLORS["vpl"])
    ax1.set_xlabel("Sigma scale")
    ax1.set_ylabel("Protection level")
    ax1.set_title("PL grows with sigma")
    ax1.grid(alpha=0.25)
    ax1.legend(frameon=False)
    ax2.plot(x, trace, marker="o", color=COLORS["gnss"])
    ax2.set_xlabel("Sigma scale")
    ax2.set_ylabel("trace(lambda_gnss)")
    ax2.set_title("Information decreases with sigma")
    ax2.grid(alpha=0.25)
    save(fig, artifact_dir / "gnss_sigma_sweep_pl.png")


def plot_current_vs_advisory(artifact_dir: Path) -> None:
    rows = read_csv(artifact_dir / "current_advisory_separation.csv")
    labels = [str(row["case_id"]) for row in rows]
    x = list(range(len(labels)))
    width = 0.34
    current_h = [row["current_hpl"] for row in rows]
    selected_h = [row["selected_hpl"] for row in rows]
    current_v = [row["current_vpl"] for row in rows]
    selected_v = [row["selected_vpl"] for row in rows]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.8))
    ax1.bar([v - width / 2 for v in x], current_h, width, label="current_hpl",
            color=COLORS["muted"])
    ax1.bar([v + width / 2 for v in x], selected_h, width, label="selected_hpl",
            color=COLORS["hpl"])
    ax2.bar([v - width / 2 for v in x], current_v, width, label="current_vpl",
            color=COLORS["muted"])
    ax2.bar([v + width / 2 for v in x], selected_v, width, label="selected_vpl",
            color=COLORS["vpl"])
    for ax in (ax1, ax2):
        ax.set_xticks(x, labels, rotation=20, ha="right")
        ax.set_yscale("log")
        ax.grid(axis="y", alpha=0.25)
        ax.legend(frameon=False)
    ax1.set_title("Horizontal PL is not copied from current")
    ax2.set_title("Vertical PL is not copied from current")
    save(fig, artifact_dir / "current_vs_advisory_isolated.png")


def plot_gnss_occlusion_pl(artifact_dir: Path) -> None:
    path = artifact_dir / "gnss_occlusion_pl_degradation.csv"
    if not path.exists():
        return
    rows = read_csv(path)
    labels = [str(row["case_id"]) for row in rows]
    metrics = ["pdop", "hpl", "vpl"]
    x = list(range(len(labels)))
    width = 0.24
    fig, ax = plt.subplots(figsize=(7.8, 4.8))
    for idx, metric in enumerate(metrics):
        vals = [row[metric] for row in rows]
        offset = (idx - 1) * width
        ax.bar([v + offset for v in x], vals, width, label=metric.upper())
    ax.set_xticks(x, labels)
    ax.set_ylabel("Metric value")
    ax.set_title("GNSS occlusion degrades PDOP/HPL/VPL")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    save(fig, artifact_dir / "gnss_occlusion_pl_degradation.png")


def plot_lidar_corridor(artifact_dir: Path) -> None:
    rows = read_csv(artifact_dir / "lidar_corridor_degeneracy.csv")
    labels = [str(row["case_id"]) for row in rows]
    x = list(range(len(labels)))

    fig, ax = plt.subplots(figsize=(7.8, 5.2))
    ax.axhline(-2, color=COLORS["lidar"], lw=4, alpha=0.65)
    ax.axhline(2, color=COLORS["lidar"], lw=4, alpha=0.65)
    ax.arrow(-4, 0, 8, 0, head_width=0.25, head_length=0.35,
             color=COLORS["warn"], length_includes_head=True)
    ax.scatter([0], [0], s=90, color=COLORS["bad"], label="query")
    for x0 in [-3, -1.5, 0, 1.5, 3]:
        ax.arrow(x0, -2, 0, 0.8, head_width=0.12, color=COLORS["muted"],
                 length_includes_head=True)
        ax.arrow(x0, 2, 0, -0.8, head_width=0.12, color=COLORS["muted"],
                 length_includes_head=True)
    ax.text(0, 0.25, "corridor axis X", ha="center", color=COLORS["warn"])
    ax.set_xlim(-4.5, 4.5)
    ax.set_ylim(-3, 3)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Corridor geometry: normals constrain cross-corridor Y")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.grid(alpha=0.18)
    save(fig, artifact_dir / "lidar_corridor_geometry_topdown.png")

    eig_labels = ["min", "mid", "max"]
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    width = 0.24
    for i, eig_name in enumerate(["min_eig", "mid_eig", "max_eig"]):
        vals = [row[eig_name] for row in rows]
        ax.bar([v + (i - 1) * width for v in x], vals, width, label=eig_labels[i])
    ax.set_xticks(x, labels)
    ax.set_yscale("symlog", linthresh=1e-6)
    ax.set_ylabel("Eigenvalue")
    ax.set_title("LiDAR corridor FIM eigenvalues")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    save(fig, artifact_dir / "lidar_corridor_eigenvalues.png")

    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    width = 0.24
    for i, axis in enumerate(["lambda_xx", "lambda_yy", "lambda_zz"]):
        vals = [row[axis] for row in rows]
        ax.bar([v + (i - 1) * width for v in x], vals, width, label=axis)
    ax.set_xticks(x, labels)
    ax.set_ylabel("Axis information")
    ax.set_title("LiDAR corridor weakens along-corridor information")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    save(fig, artifact_dir / "lidar_corridor_axis_information.png")


def plot_primitive_generation_curves(artifact_dir: Path) -> None:
    rows = read_csv(artifact_dir / "lidar_primitive_generation_parameters.csv")
    experiments = ["pca_radius", "pca_min_support", "pca_voxel_sample",
                   "cloud_normals_first"]
    fig, axes = plt.subplots(2, 2, figsize=(10, 7.2))
    for ax, exp in zip(axes.flatten(), experiments):
        exp_rows = [row for row in rows if row["experiment"] == exp]
        x = [row["param_value"] for row in exp_rows]
        y = [row["primitive_count"] for row in exp_rows]
        ax.plot(x, y, marker="o", color=COLORS["lidar"])
        ax.set_title(exp)
        ax.set_xlabel("parameter value")
        ax.set_ylabel("primitive count")
        ax.grid(alpha=0.25)
    save(fig, artifact_dir / "lidar_primitive_generation_parameter_curves.png")


def plot_fusion_gate(artifact_dir: Path) -> None:
    rows = read_csv(artifact_dir / "fusion_gate_safety.csv")
    labels = [str(row["case_id"]) for row in rows]
    x = list(range(len(labels)))
    width = 0.22
    metrics = [("gnss_hpl", "GNSS HPL"), ("fused_hpl", "raw fused HPL"),
               ("selected_hpl", "selected HPL")]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.2, 4.8))
    for i, (key, label) in enumerate(metrics):
        vals = [row[key] for row in rows]
        ax1.bar([v + (i - 1) * width for v in x], vals, width, label=label)
    metrics_v = [("gnss_vpl", "GNSS VPL"), ("fused_vpl", "raw fused VPL"),
                 ("selected_vpl", "selected VPL")]
    for i, (key, label) in enumerate(metrics_v):
        vals = [row[key] for row in rows]
        ax2.bar([v + (i - 1) * width for v in x], vals, width, label=label)
    for ax in (ax1, ax2):
        ax.set_xticks(x, labels, rotation=20, ha="right")
        ax.grid(axis="y", alpha=0.25)
        ax.legend(frameon=False, fontsize=8)
    ax1.set_title("Fusion gate: horizontal PL")
    ax2.set_title("Fusion gate: vertical PL")
    save(fig, artifact_dir / "fusion_gate_selected_vs_gnss.png")


def plot_fusion_lambda(artifact_dir: Path) -> None:
    data = json.loads((artifact_dir / "fusion_lambda_matrices.json").read_text())
    names = ["lambda_prior", "lambda_gnss", "lambda_lidar", "lambda_pred",
             "lambda_error"]
    fig, axes = plt.subplots(1, 5, figsize=(14, 3.6))
    vmax = max(abs(v) for name in names[:-1] for row in data[name] for v in row)
    for ax, name in zip(axes, names):
        matrix = data[name]
        limit = max(vmax, 1e-9) if name != "lambda_error" else max(
            max(abs(v) for row in matrix for v in row), 1e-12)
        im = ax.imshow(matrix, cmap="coolwarm", vmin=-limit, vmax=limit)
        ax.set_title(name.replace("lambda_", "Λ"))
        ax.set_xticks([0, 1, 2], ["x", "y", "z"])
        ax.set_yticks([0, 1, 2], ["x", "y", "z"])
        for i in range(3):
            for j in range(3):
                ax.text(j, i, f"{matrix[i][j]:.2g}", ha="center", va="center",
                        fontsize=7)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    save(fig, artifact_dir / "fusion_lambda_heatmaps.png")

    eig_names = ["eig_prior", "eig_gnss", "eig_lidar", "eig_pred"]
    labels = ["prior", "gnss", "lidar", "pred"]
    x = list(range(len(labels)))
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    width = 0.24
    for i in range(3):
        vals = [data[name][i] for name in eig_names]
        ax.bar([v + (i - 1) * width for v in x], vals, width,
               label=f"eig {i}")
    ax.set_xticks(x, labels)
    ax.set_yscale("log")
    ax.set_ylabel("Eigenvalue")
    ax.set_title("Fusion lambda eigenvalues")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    save(fig, artifact_dir / "fusion_lambda_eigenvalues.png")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact-dir", type=Path, required=True)
    args = parser.parse_args()
    artifact_dir = args.artifact_dir
    plot_gnss_geometry_sweep(artifact_dir)
    plot_gnss_geometry_skyplots(artifact_dir)
    plot_gnss_sigma_sweep(artifact_dir)
    plot_current_vs_advisory(artifact_dir)
    plot_gnss_occlusion_pl(artifact_dir)
    plot_lidar_corridor(artifact_dir)
    plot_primitive_generation_curves(artifact_dir)
    plot_fusion_gate(artifact_dir)
    plot_fusion_lambda(artifact_dir)


if __name__ == "__main__":
    main()
