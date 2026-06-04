#!/usr/bin/env python3
import csv
import math
import os


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "results", "araim_validation")
POINT_METRICS = os.path.join(OUT, "lidar_pointcloud_fim_pl_metrics.csv")
BLOCK_METRICS = os.path.join(OUT, "lidar_block_fault_metrics.csv")
SCENARIOS = ["feature_rich", "corridor", "sparse"]
LABELS = {
    "feature_rich": "Feature-rich",
    "corridor": "Corridor",
    "sparse": "Sparse",
}


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def as_float(row, key):
    value = row[key]
    if value == "":
        return math.nan
    return float(value)


def scenario_rows(rows):
    by_name = {row["scenario"]: row for row in rows}
    return [by_name[name] for name in SCENARIOS]


def positive(values):
    return [max(float(v), 1.0e-12) for v in values]


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, dpi=220, bbox_inches="tight")
    print("wrote", path)


def grouped_bars(ax, x_labels, series, ylabel, logy=False):
    width = 0.78 / max(len(series), 1)
    centers = list(range(len(x_labels)))
    for index, (label, values) in enumerate(series):
        offsets = [x - 0.39 + width / 2.0 + index * width for x in centers]
        ax.bar(offsets, positive(values) if logy else values,
               width=width, label=label)
    ax.set_xticks(centers)
    ax.set_xticklabels(x_labels)
    ax.set_ylabel(ylabel)
    if logy:
        ax.set_yscale("log")
    ax.grid(axis="y", which="both", alpha=0.25)
    ax.legend()


def plot_eigenvalues(rows, plt):
    labels = [LABELS[row["scenario"]] for row in rows]
    series = [
        ("lambda_min", [as_float(row, "lambda_min") for row in rows]),
        ("lambda_mid", [as_float(row, "lambda_mid") for row in rows]),
        ("lambda_max", [as_float(row, "lambda_max") for row in rows]),
    ]
    fig, ax = plt.subplots(figsize=(8.2, 4.5))
    grouped_bars(ax, labels, series, "FIM eigenvalue (log scale)", logy=True)
    ax.set_title("FIM Eigenvalue Spectrum")
    save(fig, "lidar_fim_eigenvalue_spectrum.png")
    plt.close(fig)


def plot_condition(rows, plt):
    labels = [LABELS[row["scenario"]] for row in rows]
    values = [as_float(row, "fim_condition") for row in rows]
    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    ax.bar(labels, positive(values), color=["#4c78a8", "#f58518", "#54a24b"])
    ax.set_yscale("log")
    ax.set_ylabel("FIM condition number (log scale)")
    ax.set_title("Geometry Degeneracy From FIM Condition Number")
    ax.grid(axis="y", which="both", alpha=0.25)
    save(fig, "lidar_fim_condition_number.png")
    plt.close(fig)


def plot_directional_pl(rows, plt):
    labels = [LABELS[row["scenario"]] for row in rows]
    series = [
        ("PL_E", [as_float(row, "pl_e") for row in rows]),
        ("PL_N", [as_float(row, "pl_n") for row in rows]),
        ("PL_U", [as_float(row, "pl_u") for row in rows]),
    ]
    fig, ax = plt.subplots(figsize=(8.2, 4.5))
    grouped_bars(ax, labels, series, "Directional PL [m] (log scale)",
                 logy=True)
    ax.set_title("Directional LiDAR Protection Levels")
    save(fig, "lidar_directional_pl.png")
    plt.close(fig)


def plot_hpl_vpl(rows, plt):
    labels = [LABELS[row["scenario"]] for row in rows]
    series = [
        ("HPL", [as_float(row, "hpl") for row in rows]),
        ("VPL", [as_float(row, "vpl") for row in rows]),
    ]
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    grouped_bars(ax, labels, series, "Protection level [m] (log scale)",
                 logy=True)
    ax.set_title("Final LiDAR HPL / VPL")
    save(fig, "lidar_hpl_vpl_comparison.png")
    plt.close(fig)


def plot_condition_vs_pl(rows, plt):
    conditions = positive([as_float(row, "fim_condition") for row in rows])
    hpl = positive([as_float(row, "hpl") for row in rows])
    vpl = positive([as_float(row, "vpl") for row in rows])
    labels = [LABELS[row["scenario"]] for row in rows]
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    ax.scatter(conditions, hpl, marker="o", s=70, label="HPL")
    ax.scatter(conditions, vpl, marker="s", s=70, label="VPL")
    for x, y_h, y_v, label in zip(conditions, hpl, vpl, labels):
        ax.annotate(label, (x, y_h), textcoords="offset points",
                    xytext=(6, 6), fontsize=8)
        ax.annotate(label, (x, y_v), textcoords="offset points",
                    xytext=(6, -12), fontsize=8)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("FIM condition number (log scale)")
    ax.set_ylabel("Protection level [m] (log scale)")
    ax.set_title("FIM Condition Number vs LiDAR PL")
    ax.grid(which="both", alpha=0.25)
    ax.legend()
    save(fig, "lidar_condition_vs_pl.png")
    plt.close(fig)


def plot_heatmaps(rows, plt):
    matrices = []
    max_abs = 1.0e-12
    for row in rows:
        matrix = [
            [as_float(row, "fim_00"), as_float(row, "fim_01"), as_float(row, "fim_02")],
            [as_float(row, "fim_10"), as_float(row, "fim_11"), as_float(row, "fim_12")],
            [as_float(row, "fim_20"), as_float(row, "fim_21"), as_float(row, "fim_22")],
        ]
        matrices.append(matrix)
        max_abs = max(max_abs, max(abs(v) for line in matrix for v in line))

    fig, axes = plt.subplots(1, 3, figsize=(10.8, 3.6))
    image = None
    for ax, row, matrix in zip(axes, rows, matrices):
        image = ax.imshow(matrix, cmap="RdBu_r", vmin=-max_abs, vmax=max_abs)
        ax.set_title(LABELS[row["scenario"]])
        ax.set_xticks([0, 1, 2])
        ax.set_yticks([0, 1, 2])
        ax.set_xticklabels(["E", "N", "U"])
        ax.set_yticklabels(["E", "N", "U"])
        for i in range(3):
            for j in range(3):
                ax.text(j, i, f"{matrix[i][j]:.2g}", ha="center",
                        va="center", fontsize=8)
    if image is not None:
        cbar = fig.colorbar(image, ax=axes, shrink=0.82, pad=0.03)
        cbar.set_label("FIM entry")
    fig.suptitle("LiDAR FIM Heatmaps")
    save(fig, "lidar_fim_heatmaps.png")
    plt.close(fig)


def plot_block_hpl_vpl(row, plt):
    labels = ["Clean", "Bad block"]
    series = [
        ("HPL", [as_float(row, "clean_hpl"), as_float(row, "bad_hpl")]),
        ("VPL", [as_float(row, "clean_vpl"), as_float(row, "bad_vpl")]),
    ]
    fig, ax = plt.subplots(figsize=(6.8, 4.2))
    grouped_bars(ax, labels, series, "Protection level [m]", logy=False)
    ax.set_title("Clean vs Bad LiDAR Block HPL / VPL")
    save(fig, "lidar_block_fault_hpl_vpl.png")
    plt.close(fig)


def plot_block_margin(row, plt):
    block_id = int(float(row["bad_block_id"]))
    labels = [f"Block {block_id}"]
    series = [
        ("|d_E|", [as_float(row, "abs_d_e")]),
        ("T_E", [as_float(row, "t_e")]),
    ]
    fig, ax = plt.subplots(figsize=(5.8, 4.0))
    grouped_bars(ax, labels, series, "E-direction detection metric [m]",
                 logy=False)
    ax.set_title("Block-Level Detection Margin")
    detected = int(float(row["fault_detected"])) == 1
    ax.text(0, max(as_float(row, "abs_d_e"), as_float(row, "t_e")) * 1.05,
            "fault_detected=true" if detected else "fault_detected=false",
            ha="center", va="bottom", fontsize=9)
    save(fig, "lidar_block_fault_detection_margin.png")
    plt.close(fig)


def main():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib unavailable; cannot generate plots: {exc}")

    point_rows = scenario_rows(load_rows(POINT_METRICS))
    block_rows = load_rows(BLOCK_METRICS)
    if not block_rows:
        raise SystemExit(f"missing block fault metrics in {BLOCK_METRICS}")

    plot_eigenvalues(point_rows, plt)
    plot_condition(point_rows, plt)
    plot_directional_pl(point_rows, plt)
    plot_hpl_vpl(point_rows, plt)
    plot_condition_vs_pl(point_rows, plt)
    plot_heatmaps(point_rows, plt)
    plot_block_hpl_vpl(block_rows[0], plt)
    plot_block_margin(block_rows[0], plt)


if __name__ == "__main__":
    main()
