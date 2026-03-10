#!/usr/bin/env python3
"""
IAP GNSS Factor Debug Visualizer
=================================
Reads /tmp/iap_gnss_factor_debug.csv (generated when IAP_GNSS_DEBUG_CSV=1)
and produces convergence / residual diagnostic plots.

Usage:
    python3 plot_gnss_factor_debug.py [--csv /tmp/iap_gnss_factor_debug.csv]

Plots generated:
  1. PR & Doppler RMS over time  (overall convergence)
  2. Per-satellite PR residual time series
  3. Per-satellite Doppler residual time series
  4. Normalized residual histogram  (should be ~N(0,1) if noise model is correct)
  5. Residual vs elevation  (checks elevation-dependent model)
  6. Per-constellation box plot
  7. Clock bias & drift over time
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # non-interactive backend; remove if you want GUI
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Normalize column names (strip whitespace)
    df.columns = [c.strip() for c in df.columns]
    # Relative time from first stamp
    df["t"] = df["stamp"] - df["stamp"].iloc[0]
    # Satellite label: e.g. "G12", "E05"
    df["sat_label"] = df["constellation"] + df["sat_id"].astype(str).str.zfill(2)
    return df


def plot_rms_over_time(df: pd.DataFrame, out_dir: Path):
    """Plot 1: PR & Doppler RMS per epoch."""
    fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)

    for ftype, ax, unit, color in [("PR", axes[0], "m", "tab:blue"),
                                    ("DOP", axes[1], "m/s", "tab:orange")]:
        sub = df[df["factor_type"] == ftype]
        if sub.empty:
            ax.set_title(f"{ftype}: no data")
            continue
        # RMS per epoch (diag_n)
        grp = sub.groupby("diag_n").agg(
            t=("t", "first"),
            rms=("residual", lambda x: np.sqrt((x**2).mean())),
            n=("residual", "count"),
        ).reset_index()
        ax.plot(grp["t"], grp["rms"], "-o", ms=2, color=color, label=f"{ftype} RMS")
        ax.set_ylabel(f"RMS [{unit}]")
        ax.set_title(f"{ftype} RMS over time ({grp['n'].iloc[0]} sats/epoch)")
        ax.legend()
        ax.grid(True, alpha=0.3)

    axes[1].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(out_dir / "01_rms_over_time.png", dpi=150)
    plt.close(fig)
    print(f"  [1/7] RMS over time → {out_dir / '01_rms_over_time.png'}")


def plot_per_sat_pr(df: pd.DataFrame, out_dir: Path):
    """Plot 2: Per-satellite pseudorange residual time series."""
    sub = df[df["factor_type"] == "PR"]
    if sub.empty:
        print("  [2/7] Skipped: no PR data")
        return
    sats = sorted(sub["sat_label"].unique())
    n_sats = len(sats)
    ncols = 4
    nrows = max(1, (n_sats + ncols - 1) // ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 2.5 * nrows),
                             sharex=True, sharey=True, squeeze=False)
    for idx, sat in enumerate(sats):
        ax = axes[idx // ncols][idx % ncols]
        s = sub[sub["sat_label"] == sat]
        ax.plot(s["t"], s["residual"], "-", lw=0.6, alpha=0.8)
        ax.axhline(0, color="k", lw=0.3)
        ax.set_title(sat, fontsize=9)
        ax.tick_params(labelsize=7)
    # Hide unused subplots
    for idx in range(n_sats, nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)
    fig.suptitle("Per-satellite PR residual [m]", fontsize=13)
    fig.supxlabel("Time [s]")
    fig.supylabel("Residual [m]")
    fig.tight_layout()
    fig.savefig(out_dir / "02_pr_per_satellite.png", dpi=150)
    plt.close(fig)
    print(f"  [2/7] Per-sat PR → {out_dir / '02_pr_per_satellite.png'}")


def plot_per_sat_dop(df: pd.DataFrame, out_dir: Path):
    """Plot 3: Per-satellite Doppler residual time series."""
    sub = df[df["factor_type"] == "DOP"]
    if sub.empty:
        print("  [3/7] Skipped: no DOP data")
        return
    sats = sorted(sub["sat_label"].unique())
    n_sats = len(sats)
    ncols = 4
    nrows = max(1, (n_sats + ncols - 1) // ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 2.5 * nrows),
                             sharex=True, sharey=True, squeeze=False)
    for idx, sat in enumerate(sats):
        ax = axes[idx // ncols][idx % ncols]
        s = sub[sub["sat_label"] == sat]
        ax.plot(s["t"], s["residual"], "-", lw=0.6, alpha=0.8, color="tab:orange")
        ax.axhline(0, color="k", lw=0.3)
        ax.set_title(sat, fontsize=9)
        ax.tick_params(labelsize=7)
    for idx in range(n_sats, nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)
    fig.suptitle("Per-satellite Doppler residual [m/s]", fontsize=13)
    fig.supxlabel("Time [s]")
    fig.supylabel("Residual [m/s]")
    fig.tight_layout()
    fig.savefig(out_dir / "03_dop_per_satellite.png", dpi=150)
    plt.close(fig)
    print(f"  [3/7] Per-sat DOP → {out_dir / '03_dop_per_satellite.png'}")


def plot_normalized_histogram(df: pd.DataFrame, out_dir: Path):
    """Plot 4: Normalized residual histogram — should be ~N(0,1)."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ftype, ax, color in [("PR", axes[0], "tab:blue"), ("DOP", axes[1], "tab:orange")]:
        sub = df[df["factor_type"] == ftype]
        if sub.empty:
            ax.set_title(f"{ftype}: no data")
            continue
        nr = sub["normalized_residual"].dropna()
        ax.hist(nr, bins=80, density=True, alpha=0.7, color=color, edgecolor="white", lw=0.3)
        # Overlay N(0,1)
        x = np.linspace(-6, 6, 200)
        ax.plot(x, np.exp(-x**2 / 2) / np.sqrt(2 * np.pi), "k--", lw=1.5, label="N(0,1)")
        ax.set_xlabel("Normalized residual (r/σ)")
        ax.set_ylabel("Density")
        ax.set_title(f"{ftype}: normalized residual histogram\n"
                     f"mean={nr.mean():.2f}  std={nr.std():.2f}  N={len(nr)}")
        ax.legend()
        ax.set_xlim(-6, 6)
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "04_normalized_histogram.png", dpi=150)
    plt.close(fig)
    print(f"  [4/7] Normalized histogram → {out_dir / '04_normalized_histogram.png'}")


def plot_residual_vs_elevation(df: pd.DataFrame, out_dir: Path):
    """Plot 5: Residual magnitude vs elevation angle."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ftype, ax, color, unit in [("PR", axes[0], "tab:blue", "m"),
                                    ("DOP", axes[1], "tab:orange", "m/s")]:
        sub = df[df["factor_type"] == ftype]
        if sub.empty:
            ax.set_title(f"{ftype}: no data")
            continue
        ax.scatter(sub["elevation_deg"], sub["residual"].abs(), s=2, alpha=0.15, color=color)
        # Binned median
        bins = np.arange(10, 95, 5)
        sub_copy = sub.copy()
        sub_copy["elev_bin"] = pd.cut(sub_copy["elevation_deg"], bins)
        med = sub_copy.groupby("elev_bin", observed=True)["residual"].apply(lambda x: x.abs().median())
        bin_centers = [(b.left + b.right) / 2 for b in med.index]
        ax.plot(bin_centers, med.values, "k-o", ms=4, lw=1.5, label="Median |r|")
        ax.set_xlabel("Elevation [°]")
        ax.set_ylabel(f"|Residual| [{unit}]")
        ax.set_title(f"{ftype}: residual vs elevation")
        ax.legend()
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "05_residual_vs_elevation.png", dpi=150)
    plt.close(fig)
    print(f"  [5/7] Residual vs elevation → {out_dir / '05_residual_vs_elevation.png'}")


def plot_constellation_boxplot(df: pd.DataFrame, out_dir: Path):
    """Plot 6: Per-constellation residual box plot."""
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    constellation_map = {"G": "GPS", "R": "GLO", "E": "GAL", "C": "BDS"}
    for ftype, ax, unit in [("PR", axes[0], "m"), ("DOP", axes[1], "m/s")]:
        sub = df[df["factor_type"] == ftype].copy()
        if sub.empty:
            ax.set_title(f"{ftype}: no data")
            continue
        sub["sys"] = sub["constellation"].map(constellation_map).fillna(sub["constellation"])
        systems = sorted(sub["sys"].unique())
        data = [sub[sub["sys"] == s]["residual"].values for s in systems]
        bp = ax.boxplot(data, labels=systems, showfliers=False, patch_artist=True)
        colors = {"GPS": "#4285F4", "GLO": "#EA4335", "GAL": "#FBBC05", "BDS": "#34A853"}
        for patch, sys in zip(bp["boxes"], systems):
            patch.set_facecolor(colors.get(sys, "gray"))
            patch.set_alpha(0.6)
        ax.axhline(0, color="k", lw=0.3)
        ax.set_ylabel(f"Residual [{unit}]")
        ax.set_title(f"{ftype}: residual by constellation")
        ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_dir / "06_constellation_boxplot.png", dpi=150)
    plt.close(fig)
    print(f"  [6/7] Constellation boxplot → {out_dir / '06_constellation_boxplot.png'}")


def plot_clock_state(df: pd.DataFrame, out_dir: Path):
    """Plot 7: Clock bias and drift over time."""
    # One row per epoch is enough
    clk = df.drop_duplicates(subset=["diag_n"])[["t", "clk_bias_m", "clk_drift_ms"]].dropna()
    if clk.empty:
        print("  [7/7] Skipped: no clock data")
        return
    fig, axes = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
    axes[0].plot(clk["t"], clk["clk_bias_m"] / 1e3, "-", lw=1, color="tab:purple")
    axes[0].set_ylabel("Clock bias [km]")
    axes[0].set_title("Receiver clock bias")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(clk["t"], clk["clk_drift_ms"], "-", lw=1, color="tab:red")
    axes[1].set_ylabel("Clock drift [m/s]")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_title("Receiver clock drift")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "07_clock_state.png", dpi=150)
    plt.close(fig)
    print(f"  [7/7] Clock state → {out_dir / '07_clock_state.png'}")


# ═════════════════════════════════════════════════════════════════════════════
#  Data Analysis & Anomaly Detection
# ═════════════════════════════════════════════════════════════════════════════

# ANSI colors for terminal output
class _C:
    RED    = "\033[91m"
    YELLOW = "\033[93m"
    GREEN  = "\033[92m"
    CYAN   = "\033[96m"
    BOLD   = "\033[1m"
    RESET  = "\033[0m"


def _warn(msg: str) -> str:
    return f"  {_C.YELLOW}⚠ WARNING:{_C.RESET} {msg}"


def _alert(msg: str) -> str:
    return f"  {_C.RED}✘ ALERT:{_C.RESET}   {msg}"


def _ok(msg: str) -> str:
    return f"  {_C.GREEN}✔ OK:{_C.RESET}      {msg}"


def _info(msg: str) -> str:
    return f"  {_C.CYAN}ℹ INFO:{_C.RESET}    {msg}"


def analyze_and_report(df: pd.DataFrame, out_dir: Path) -> int:
    """
    Run comprehensive analysis on factor residuals.
    Returns the number of alerts (critical issues).
    """
    lines: list[str] = []
    alerts = 0
    warnings = 0

    lines.append("")
    lines.append(f"{_C.BOLD}{'═' * 72}{_C.RESET}")
    lines.append(f"{_C.BOLD}  GNSS Factor Diagnostic Report{_C.RESET}")
    lines.append(f"{_C.BOLD}{'═' * 72}{_C.RESET}")

    pr = df[df["factor_type"] == "PR"]
    dop = df[df["factor_type"] == "DOP"]

    # ── 1. Overall RMS statistics ──────────────────────────────────────────
    lines.append(f"\n{_C.BOLD}[1] Overall Residual Statistics{_C.RESET}")
    for label, sub, unit, rms_good, rms_warn in [
        ("Pseudorange (PR)", pr, "m", 10.0, 20.0),
        ("Doppler (DOP)", dop, "m/s", 1.0, 2.0),
    ]:
        if sub.empty:
            lines.append(f"  {label}: no data")
            continue
        rms = np.sqrt((sub["residual"] ** 2).mean())
        mean = sub["residual"].mean()
        std = sub["residual"].std()
        med = sub["residual"].median()
        p95 = sub["residual"].abs().quantile(0.95)
        lines.append(f"  {label}:")
        lines.append(f"    RMS={rms:.3f} {unit}  mean={mean:.3f}  std={std:.3f}  "
                     f"median={med:.3f}  |r|_95%={p95:.3f} {unit}")
        if rms < rms_good:
            lines.append(_ok(f"RMS {rms:.2f} {unit} — healthy"))
        elif rms < rms_warn:
            lines.append(_warn(f"RMS {rms:.2f} {unit} — elevated (expected <{rms_good} {unit} "
                               f"with all corrections)"))
            warnings += 1
        else:
            lines.append(_alert(f"RMS {rms:.2f} {unit} — too large! "
                                f"Check iono/tropo corrections, satellite clock, transit time"))
            alerts += 1

    # ── 2. Convergence trend ───────────────────────────────────────────────
    lines.append(f"\n{_C.BOLD}[2] Convergence Trend{_C.RESET}")
    if not pr.empty:
        grp = pr.groupby("diag_n")["residual"].apply(lambda x: np.sqrt((x**2).mean()))
        first_10 = grp.iloc[:min(10, len(grp))].mean()
        last_10 = grp.iloc[-min(10, len(grp)):].mean()
        change_pct = (last_10 - first_10) / first_10 * 100 if first_10 > 0 else 0
        lines.append(f"  PR RMS: first 10 epochs avg={first_10:.2f}m → "
                     f"last 10 epochs avg={last_10:.2f}m  ({change_pct:+.1f}%)")
        if last_10 < first_10 * 0.8:
            lines.append(_ok("PR residuals are converging"))
        elif last_10 > first_10 * 1.2:
            lines.append(_alert("PR residuals are DIVERGING — check factor graph stability"))
            alerts += 1
        else:
            lines.append(_info("PR residuals roughly stable (no clear convergence)"))

    # ── 3. Noise model consistency (normalized residuals) ──────────────────
    lines.append(f"\n{_C.BOLD}[3] Noise Model Consistency (normalized residual ≈ N(0,1)?){_C.RESET}")
    for label, sub, in [("PR", pr), ("DOP", dop)]:
        if sub.empty:
            continue
        nr = sub["normalized_residual"].dropna()
        nr_mean = nr.mean()
        nr_std = nr.std()
        lines.append(f"  {label}: mean={nr_mean:.3f}  std={nr_std:.3f}  "
                     f"(ideal: mean≈0, std≈1)")
        if abs(nr_mean) > 1.0:
            lines.append(_alert(f"{label} systematic bias: mean(r/σ)={nr_mean:.2f} — "
                                "measurement model has uncompensated error "
                                "(iono? tropo? TGD? ISB?)"))
            alerts += 1
        elif abs(nr_mean) > 0.3:
            lines.append(_warn(f"{label} slight bias: mean(r/σ)={nr_mean:.2f}"))
            warnings += 1
        if nr_std > 2.0:
            lines.append(_alert(f"{label} noise model too TIGHT: std(r/σ)={nr_std:.2f} "
                                "(should ≈1.0). Increase σ or fix unmodeled errors"))
            alerts += 1
        elif nr_std > 1.5:
            lines.append(_warn(f"{label} noise model slightly tight: std(r/σ)={nr_std:.2f}"))
            warnings += 1
        elif nr_std < 0.3:
            lines.append(_warn(f"{label} noise model too LOOSE: std(r/σ)={nr_std:.2f} — "
                               "σ overestimated, factors have little influence"))
            warnings += 1
        else:
            lines.append(_ok(f"{label} noise model reasonable (std={nr_std:.2f})"))

    # ── 4. Per-satellite outlier detection ─────────────────────────────────
    lines.append(f"\n{_C.BOLD}[4] Per-Satellite Anomaly Detection{_C.RESET}")
    bad_sats_pr = []
    bad_sats_dop = []

    if not pr.empty:
        sat_stats = pr.groupby("sat_label").agg(
            rms=("residual", lambda x: np.sqrt((x**2).mean())),
            mean=("residual", "mean"),
            std=("residual", "std"),
            count=("residual", "count"),
            elev=("elevation_deg", "mean"),
            pct_outlier=("normalized_residual", lambda x: (x.abs() > 3).mean() * 100),
        ).sort_values("rms", ascending=False)

        overall_rms = np.sqrt((pr["residual"] ** 2).mean())
        lines.append(f"  PR per-satellite (overall RMS={overall_rms:.2f}m):")

        # Table header
        lines.append(f"    {'SAT':<6} {'RMS':>8} {'Mean':>8} {'Std':>8} "
                     f"{'Elev°':>6} {'N':>5} {'%|r/σ|>3':>9}  Status")
        lines.append(f"    {'─'*6} {'─'*8} {'─'*8} {'─'*8} {'─'*6} {'─'*5} {'─'*9}  {'─'*12}")

        for sat, row in sat_stats.iterrows():
            status = ""
            is_bad = False
            if row["rms"] > overall_rms * 2.0:
                status = f"{_C.RED}✘ HIGH RMS{_C.RESET}"
                is_bad = True
            elif row["pct_outlier"] > 20:
                status = f"{_C.YELLOW}⚠ OUTLIERS{_C.RESET}"
                is_bad = True
            elif abs(row["mean"]) > overall_rms:
                status = f"{_C.YELLOW}⚠ BIASED{_C.RESET}"
                is_bad = True
            else:
                status = f"{_C.GREEN}✔{_C.RESET}"

            lines.append(f"    {sat:<6} {row['rms']:>8.2f} {row['mean']:>8.2f} "
                         f"{row['std']:>8.2f} {row['elev']:>6.1f} {row['count']:>5.0f} "
                         f"{row['pct_outlier']:>8.1f}%  {status}")
            if is_bad:
                bad_sats_pr.append(sat)

        if bad_sats_pr:
            lines.append(_alert(f"Anomalous PR satellites: {', '.join(bad_sats_pr)}"))
            alerts += 1
        else:
            lines.append(_ok("All satellites within normal PR range"))

    # ── 5. Per-constellation analysis (ISB check) ─────────────────────────
    lines.append(f"\n{_C.BOLD}[5] Per-Constellation Analysis (Inter-System Bias check){_C.RESET}")
    constellation_map = {"G": "GPS", "R": "GLONASS", "E": "Galileo", "C": "BeiDou"}
    if not pr.empty:
        con_stats = pr.groupby("constellation").agg(
            rms=("residual", lambda x: np.sqrt((x**2).mean())),
            mean=("residual", "mean"),
            std=("residual", "std"),
            n_sats=("sat_id", "nunique"),
            count=("residual", "count"),
        ).sort_values("rms", ascending=False)

        lines.append(f"    {'System':<10} {'RMS':>8} {'Mean':>8} {'Std':>8} "
                     f"{'#Sats':>6} {'N':>7}  Status")
        lines.append(f"    {'─'*10} {'─'*8} {'─'*8} {'─'*8} {'─'*6} {'─'*7}  {'─'*20}")

        gps_mean = None
        for con, row in con_stats.iterrows():
            name = constellation_map.get(con, con)
            if con == "G":
                gps_mean = row["mean"]
            diff_str = ""
            status = f"{_C.GREEN}✔{_C.RESET}"
            if gps_mean is not None and con != "G":
                isb = row["mean"] - gps_mean
                diff_str = f" (ISB vs GPS: {isb:+.2f}m)"
                if abs(isb) > 15:
                    status = f"{_C.RED}✘ LARGE ISB{_C.RESET}"
                    alerts += 1
                elif abs(isb) > 5:
                    status = f"{_C.YELLOW}⚠ ISB{_C.RESET}"
                    warnings += 1

            lines.append(f"    {name:<10} {row['rms']:>8.2f} {row['mean']:>8.2f} "
                         f"{row['std']:>8.2f} {row['n_sats']:>6} {row['count']:>7}  "
                         f"{status}{diff_str}")

        # Check if GLONASS has much larger residuals (common — no ISB modeled)
        if "R" in con_stats.index and "G" in con_stats.index:
            glo_rms = con_stats.loc["R", "rms"]
            gps_rms = con_stats.loc["G", "rms"]
            if glo_rms > gps_rms * 2:
                lines.append(_warn(f"GLONASS RMS ({glo_rms:.1f}m) >> GPS ({gps_rms:.1f}m) — "
                                   "consider modeling Inter-Frequency Bias (IFB) "
                                   "or increasing GLONASS σ"))
                warnings += 1

    # ── 6. Elevation-dependent analysis ────────────────────────────────────
    lines.append(f"\n{_C.BOLD}[6] Elevation-Dependent Analysis{_C.RESET}")
    if not pr.empty:
        low = pr[pr["elevation_deg"] < 30]
        high = pr[pr["elevation_deg"] >= 60]
        if len(low) > 0 and len(high) > 0:
            rms_low = np.sqrt((low["residual"] ** 2).mean())
            rms_high = np.sqrt((high["residual"] ** 2).mean())
            ratio = rms_low / rms_high if rms_high > 0 else float("inf")
            lines.append(f"  PR: low-elev (<30°) RMS={rms_low:.2f}m  "
                         f"high-elev (≥60°) RMS={rms_high:.2f}m  ratio={ratio:.2f}")
            if ratio > 4.0:
                lines.append(_warn(f"Low-elevation noise ratio={ratio:.1f}x — "
                                   "consider tighter elevation mask or better "
                                   "multipath/iono model"))
                warnings += 1
            elif ratio < 1.2:
                lines.append(_warn(f"No elevation dependence (ratio={ratio:.1f}) — "
                                   "noise model may not match reality"))
                warnings += 1
            else:
                lines.append(_ok(f"Elevation dependence normal (ratio={ratio:.1f}x)"))

    # ── 7. Clock state analysis ────────────────────────────────────────────
    lines.append(f"\n{_C.BOLD}[7] Clock State Analysis{_C.RESET}")
    clk = df.drop_duplicates(subset=["diag_n"])[["t", "diag_n", "clk_bias_m", "clk_drift_ms"]].dropna()
    if len(clk) > 5:
        # Check drift stability
        drift = clk["clk_drift_ms"]
        drift_mean = drift.mean()
        drift_std = drift.std()
        drift_range = drift.max() - drift.min()
        lines.append(f"  Clock drift: mean={drift_mean:.4f} m/s  "
                     f"std={drift_std:.4f} m/s  range={drift_range:.4f} m/s")

        # Convert to ppm for physical interpretation
        c = 299792458.0
        ppm = drift_mean / c * 1e6
        lines.append(f"  Frequency offset: {ppm:.4f} ppm  "
                     f"({drift_mean/c*1e9:.2f} ppb)")

        if drift_std > 5.0:
            lines.append(_alert(f"Clock drift unstable: std={drift_std:.2f} m/s — "
                                "possible TCXO issue or ClockBetweenFactor too loose"))
            alerts += 1
        elif drift_std > 1.0:
            lines.append(_warn(f"Clock drift variation: std={drift_std:.2f} m/s"))
            warnings += 1
        else:
            lines.append(_ok(f"Clock drift stable (std={drift_std:.4f} m/s)"))

        # Check for sudden jumps (>5 m/s between consecutive epochs)
        drift_diff = drift.diff().abs()
        jumps = drift_diff[drift_diff > 5.0]
        if len(jumps) > 0:
            lines.append(_alert(f"{len(jumps)} clock drift jump(s) >5 m/s detected — "
                                "possible solver divergence or marginalization artifact"))
            alerts += 1

        # Check bias rate of change matches drift
        if len(clk) > 10:
            dt = clk["t"].diff()
            dbias = clk["clk_bias_m"].diff()
            implied_drift = (dbias / dt).dropna()
            implied_drift = implied_drift[(dt > 0.01) & (dt < 5)]
            if len(implied_drift) > 0:
                consistency = np.abs(implied_drift.mean() - drift_mean)
                lines.append(f"  Bias→drift consistency: Δbias/Δt={implied_drift.mean():.2f} m/s "
                             f"vs est. drift={drift_mean:.2f} m/s  (diff={consistency:.2f})")
                if consistency > 5.0:
                    lines.append(_warn("Clock bias change rate ≠ estimated drift — "
                                       "possible estimation inconsistency"))
                    warnings += 1
                else:
                    lines.append(_ok("Clock bias/drift consistent"))

    # ── 8. Temporal outlier detection ──────────────────────────────────────
    lines.append(f"\n{_C.BOLD}[8] Temporal Outlier Epochs{_C.RESET}")
    if not pr.empty:
        epoch_rms = pr.groupby("diag_n").agg(
            t=("t", "first"),
            rms=("residual", lambda x: np.sqrt((x**2).mean())),
            n=("residual", "count"),
        ).reset_index()
        median_rms = epoch_rms["rms"].median()
        outlier_epochs = epoch_rms[epoch_rms["rms"] > median_rms * 2.5]
        if len(outlier_epochs) > 0:
            pct = len(outlier_epochs) / len(epoch_rms) * 100
            lines.append(_warn(f"{len(outlier_epochs)} epoch(s) ({pct:.1f}%) with PR RMS "
                               f"> 2.5× median ({median_rms:.1f}m):"))
            for _, row in outlier_epochs.head(10).iterrows():
                lines.append(f"    diag #{int(row['diag_n'])}: t={row['t']:.1f}s  "
                             f"RMS={row['rms']:.2f}m  ({int(row['n'])} sats)")
            if len(outlier_epochs) > 10:
                lines.append(f"    ... and {len(outlier_epochs) - 10} more")
            warnings += 1
        else:
            lines.append(_ok(f"No outlier epochs (all within 2.5× median={median_rms:.1f}m)"))

    # ── 9. Missing ionosphere correction check ─────────────────────────────
    lines.append(f"\n{_C.BOLD}[9] Ionosphere Correction Check{_C.RESET}")
    if not pr.empty:
        overall_mean = pr["residual"].mean()
        overall_rms = np.sqrt((pr["residual"] ** 2).mean())
        if overall_rms > 10.0 and abs(overall_mean) > 5.0:
            lines.append(_alert(f"Large systematic PR bias (mean={overall_mean:.1f}m, "
                                f"RMS={overall_rms:.1f}m) — likely missing ionospheric "
                                "correction. Check /ublox_driver/iono_params and iono log"))
            alerts += 1
        elif overall_rms > 10.0:
            lines.append(_warn(f"PR RMS={overall_rms:.1f}m elevated — "
                               "possible missing atmospheric correction"))
            warnings += 1
        else:
            lines.append(_ok(f"PR residuals within range (RMS={overall_rms:.1f}m)"))

    # ── Summary ────────────────────────────────────────────────────────────
    lines.append(f"\n{_C.BOLD}{'═' * 72}{_C.RESET}")
    if alerts > 0:
        lines.append(f"{_C.RED}{_C.BOLD}  Summary: {alerts} ALERT(s), "
                     f"{warnings} WARNING(s){_C.RESET}")
    elif warnings > 0:
        lines.append(f"{_C.YELLOW}{_C.BOLD}  Summary: {alerts} ALERT(s), "
                     f"{warnings} WARNING(s){_C.RESET}")
    else:
        lines.append(f"{_C.GREEN}{_C.BOLD}  Summary: All checks passed ✔{_C.RESET}")
    lines.append(f"{_C.BOLD}{'═' * 72}{_C.RESET}")
    lines.append("")

    # Print to terminal
    report = "\n".join(lines)
    print(report)

    # Save plain-text version (strip ANSI codes)
    import re
    ansi_re = re.compile(r"\033\[[0-9;]*m")
    plain = ansi_re.sub("", report)
    report_path = out_dir / "analysis_report.txt"
    report_path.write_text(plain)
    print(f"  Report saved → {report_path}")

    return alerts


def main():
    parser = argparse.ArgumentParser(
        description="IAP GNSS Factor Debug Visualizer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--csv", default="/tmp/iap_gnss_factor_debug.csv",
        help="Path to the debug CSV file (default: /tmp/iap_gnss_factor_debug.csv)",
    )
    parser.add_argument(
        "--out", default="/tmp/iap_gnss_plots",
        help="Output directory for plots (default: /tmp/iap_gnss_plots)",
    )
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: CSV not found: {csv_path}")
        print("  Run with IAP_GNSS_DEBUG_CSV=1 to generate it.")
        sys.exit(1)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading {csv_path} ...")
    df = load_csv(str(csv_path))
    n_epochs = df["diag_n"].nunique()
    n_pr = (df["factor_type"] == "PR").sum()
    n_dop = (df["factor_type"] == "DOP").sum()
    n_sats = df["sat_label"].nunique()
    print(f"  {len(df)} rows | {n_epochs} epochs | {n_sats} satellites | "
          f"{n_pr} PR + {n_dop} DOP factors")
    print(f"Generating plots → {out_dir}/")

    plot_rms_over_time(df, out_dir)
    plot_per_sat_pr(df, out_dir)
    plot_per_sat_dop(df, out_dir)
    plot_normalized_histogram(df, out_dir)
    plot_residual_vs_elevation(df, out_dir)
    plot_constellation_boxplot(df, out_dir)
    plot_clock_state(df, out_dir)

    print(f"\nAll plots saved to {out_dir}/")

    # ── Run analysis & anomaly detection ──
    n_alerts = analyze_and_report(df, out_dir)

    print("  Tip: open with  $BROWSER /tmp/iap_gnss_plots/01_rms_over_time.png")

    # Return non-zero exit code if critical alerts found
    if n_alerts > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
