#!/usr/bin/env python3
# IAP-RQ-200: ARAIM epoch timeline plots — Fig B1, B2, B3
# Usage: python3 tools/plot_araim_timeline.py /tmp/iap_araim.csv

import sys
import pathlib
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

OUT_DIR = pathlib.Path("/home/dev/code/ws_iap/src/iap/tools/figs")
STYLE_EPOCH = dict(marker=".", markersize=2, linewidth=0.8)


def load_data(csv_path: str):
    path = pathlib.Path(csv_path)
    if not path.exists() and not path.is_absolute():
        alt = pathlib.Path("/tmp") / path
        if alt.exists():
            path = alt
    if not path.exists():
        raise FileNotFoundError(
            f"CSV not found: {csv_path} (also tried: /tmp/{pathlib.Path(csv_path).name})"
        )

    try:
        df = pd.read_csv(path)
    except pd.errors.ParserError as e:
        print(f"[warn] CSV parse error ({e}); retrying with bad-line skipping")
        df = pd.read_csv(path, on_bad_lines="skip", engine="python")
    if "row_type" not in df.columns:
        raise ValueError("CSV missing required column: row_type")

    epochs = df[df["row_type"] == "epoch"].copy().reset_index(drop=True)
    hyps   = df[df["row_type"] == "worst_hyp"].copy().reset_index(drop=True)
    return epochs, hyps


def state_colors(states):
    cmap = {"SAFE": "#2ca02c", "SAFE_EXCLUDED": "#ff7f0e", "UNSAFE": "#d62728"}
    return [cmap.get(s, "#aaaaaa") for s in states]


# ---------------------------------------------------------------------------
# Fig B1: HPL/VPL vs HAL/VAL + IM + state machine coloring
# ---------------------------------------------------------------------------
def plot_B1(epochs, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    t = epochs["stamp"] - epochs["stamp"].iloc[0]

    # ---- State color bands ----
    for ax in axes:
        prev_state, t_start = None, t.iloc[0]
        for i, (ti, si) in enumerate(zip(t, epochs["state"])):
            if si != prev_state:
                if prev_state is not None:
                    c = state_colors([prev_state])[0]
                    ax.axvspan(t_start, ti, alpha=0.08, color=c, linewidth=0)
                prev_state, t_start = si, ti
        c = state_colors([prev_state])[0]
        ax.axvspan(t_start, t.iloc[-1], alpha=0.08, color=c, linewidth=0)

    # ---- Panel 1: HPL/VPL vs HAL/VAL ----
    ax = axes[0]
    ax.plot(t, epochs["HPL"], "b-",  label="HPL", **STYLE_EPOCH)
    ax.plot(t, epochs["VPL"], "c-",  label="VPL", **STYLE_EPOCH)
    ax.plot(t, epochs["HAL"], "r--", label="HAL", linewidth=0.9)
    ax.plot(t, epochs["VAL"], "m--", label="VAL", linewidth=0.9)
    ax.set_ylabel("Protection / Alert Level [m]")
    ax.legend(loc="upper right", fontsize=8, ncol=4)
    ax.grid(True, alpha=0.3)
    ax.set_title("Fig B1 — ARAIM Integrity Timeline")

    # ---- Panel 2: Integrity Margin ----
    ax = axes[1]
    colors_im = ["#2ca02c" if v > 0 else "#d62728" for v in epochs["IM"]]
    ax.bar(t, epochs["IM"], width=(t.iloc[1] - t.iloc[0]) if len(t) > 1 else 0.1,
           color=colors_im, alpha=0.7)
    ax.axhline(0, color="k", linewidth=0.5)
    ax.set_ylabel("IM = AL − PL [m]")
    ax.grid(True, alpha=0.3)

    # ---- Panel 3: State machine ----
    ax = axes[2]
    state_num = epochs["state"].map({"SAFE": 2, "SAFE_EXCLUDED": 1, "UNSAFE": 0})
    sc = ax.scatter(t, state_num, c=state_colors(epochs["state"]), s=4, zorder=3)
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["UNSAFE", "SAFE_EX", "SAFE"], fontsize=8)
    ax.set_ylabel("Integrity State")
    ax.set_xlabel("Time [s]")
    ax.grid(True, alpha=0.3)
    # Legend patches
    patches = [mpatches.Patch(color=c, label=s)
               for s, c in [("SAFE", "#2ca02c"), ("SAFE_EXCLUDED", "#ff7f0e"), ("UNSAFE", "#d62728")]]
    ax.legend(handles=patches, loc="lower right", fontsize=8)

    fig.tight_layout()
    for ext in ("pdf", "png"):
        path = out_dir / f"fig_B1_araim_timeline.{ext}"
        fig.savefig(path, dpi=150)
        print(f"Saved {path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Fig B2: Worst-hypothesis 3-term PL decomposition stacked bar
# ---------------------------------------------------------------------------
def plot_B2(hyps, out_dir):
    if hyps.empty or hyps["d_E"].isna().all():
        print("[B2] No worst_hyp rows — skipping Fig B2")
        return

    # For each worst_hyp row, compute per-axis 3-term breakdown
    # PL_E = |d_E| + K_fa*sigma_ss_E + K_md*(estimated sigma_k_E)
    # We use stored PL_E and back out |d_E|, K_fa*sigma_ss_E, K_md*sigma_k_E
    # Simplification: use directly d_E, sigma_ss, sigma_k from the row.
    t = np.arange(len(hyps))  # row index (worst_hyp rows)

    fig, axes = plt.subplots(1, 3, figsize=(14, 5), sharey=False)
    axes_label = ["East", "North", "Up"]
    d_cols      = ["d_E", "d_N", "d_U"]
    ss_cols     = ["sigma_ss_E", "sigma_ss_N", "sigma_ss_U"]
    k_cols      = ["sigma_k_E", "sigma_k_N", "sigma_k_U"]
    pl_cols     = ["hyp_PL_E", "hyp_PL_N", "hyp_PL_U"]

    for i, (ax, lbl, dc, sc, kc, pc) in enumerate(
            zip(axes, axes_label, d_cols, ss_cols, k_cols, pl_cols)):
        d  = hyps[dc].abs().fillna(0).values
        ss = hyps[sc].fillna(0).values
        sk = hyps[kc].fillna(0).values

        ax.bar(t, d,      label="|d_k|",       color="#1f77b4", alpha=0.85)
        ax.bar(t, ss, bottom=d,      label="K_fa·σ_ss", color="#ff7f0e", alpha=0.85)
        ax.bar(t, sk, bottom=d + ss, label="K_md·σ_k",  color="#2ca02c", alpha=0.85)
        ax.set_title(f"{lbl} worst-hyp PL decomposition")
        ax.set_xlabel("Epoch index")
        ax.set_ylabel("PL component [m]")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3, axis="y")

    fig.suptitle("Fig B2 — ARAIM 3-Term PL Decomposition (Worst Hypothesis)")
    fig.tight_layout()
    for ext in ("pdf", "png"):
        path = out_dir / f"fig_B2_araim_pl_decomp.{ext}"
        fig.savefig(path, dpi=150)
        print(f"Saved {path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
# Fig B3: n_sv / n_constellations / n_detected timeline
# ---------------------------------------------------------------------------
def plot_B3(epochs, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    t = epochs["stamp"] - epochs["stamp"].iloc[0]

    axes[0].plot(t, epochs["n_sv"],    "b-", **STYLE_EPOCH, label="n_sv (used)")
    axes[0].set_ylabel("# Satellites")
    axes[0].legend(fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title("Fig B3 — GNSS Satellite Health Overview")

    axes[1].plot(t, epochs["n_const"], "g-", **STYLE_EPOCH, label="n_constellations")
    axes[1].set_ylabel("# Constellations")
    axes[1].set_yticks([1, 2, 3, 4])
    axes[1].legend(fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].bar(t, epochs["n_det"], width=(t.iloc[1]-t.iloc[0]) if len(t)>1 else 0.1,
                color="#d62728", alpha=0.7, label="n_detected (FDE)")
    axes[2].set_ylabel("# Detected Faults")
    axes[2].set_xlabel("Time [s]")
    axes[2].legend(fontsize=8)
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    for ext in ("pdf", "png"):
        path = out_dir / f"fig_B3_satellite_health.{ext}"
        fig.savefig(path, dpi=150)
        print(f"Saved {path}")
    plt.close(fig)


# ---------------------------------------------------------------------------
def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/iap_araim.csv"
    out_dir  = pathlib.Path(sys.argv[2]) if len(sys.argv) > 2 else OUT_DIR

    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading {csv_path} …")
    epochs, hyps = load_data(csv_path)
    print(f"  {len(epochs)} epoch rows, {len(hyps)} worst_hyp rows")

    if epochs.empty:
        print("ERROR: no epoch rows found — check CSV file")
        sys.exit(1)

    plot_B1(epochs, out_dir)
    plot_B2(hyps,   out_dir)
    plot_B3(epochs, out_dir)
    print("Done — Figs B1/B2/B3 saved.")


if __name__ == "__main__":
    main()
