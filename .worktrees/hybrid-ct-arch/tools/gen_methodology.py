#!/usr/bin/env python3
"""IAP-RQ-900: Auto-generate IEEE Trans methodology chapter.

Usage:
    python3 tools/gen_methodology.py \
        --traceability docs/TRACEABILITY.md \
        --output docs/methodology/methodology.tex \
        --figure docs/figures/system_flow.pdf

Produces:
  - docs/methodology/methodology.tex   (compilable LaTeX)
  - docs/figures/system_flow.tex       (TikZ flowchart placeholder, compiled to .pdf on demand)

The generated .tex includes:
  1. System flowchart reference (Figure 1)
  2. Auto-generated module subsections from TRACEABILITY.md rows
  3. Formula skeleton per module (pulled from comment lines)
"""

import argparse
import re
import sys
import os
from datetime import datetime
from pathlib import Path


# ---------------------------------------------------------------------------
# Parsing helpers
# ---------------------------------------------------------------------------

def parse_traceability(path: str):
    """Parse TRACEABILITY.md table rows into list of dicts."""
    rows = []
    in_table = False
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.rstrip()
            if line.startswith("| Req ID") or line.startswith("|---|"):
                in_table = True
                continue
            if in_table and line.startswith("| IAP-RQ-"):
                cols = [c.strip() for c in line.split("|")]
                # cols[0] = '', cols[1] = req_id, ..., cols[7] = status
                if len(cols) >= 8:
                    rows.append({
                        "req_id":   cols[1],
                        "desc":     cols[2],
                        "idea":     cols[3],
                        "impl":     cols[4],
                        "test":     cols[5],
                        "metrics":  cols[6],
                        "status":   cols[7],
                    })
            elif in_table and not line.startswith("|"):
                in_table = False
    return rows


def escape_latex(s: str) -> str:
    """Escape special LaTeX chars in a string."""
    chars = {
        "&":  r"\&",
        "%":  r"\%",
        "$":  r"\$",
        "#":  r"\#",
        "_":  r"\_",
        "{":  r"\{",
        "}":  r"\}",
        "~":  r"\textasciitilde{}",
        "^":  r"\textasciicircum{}",
        "\\": r"\textbackslash{}",
    }
    return "".join(chars.get(c, c) for c in s)


# ---------------------------------------------------------------------------
# Section templates (formulas extracted from known REQ patterns)
# ---------------------------------------------------------------------------

_FORMULA_MAP = {
    "IAP-RQ-015": r"""
  The position covariance proxy $\Sigma_p \in \mathbb{R}^{3\times3}$ is extracted
  from the fixed-lag smoother marginal:
  \begin{equation}
    \sigma_p = \sqrt{\lambda_{\max}(\Sigma_p)}, \qquad
    \mathrm{PL}_{\mathrm{proxy}} = K \sigma_p
    \label{eq:sigma_p}
  \end{equation}
  where $K=3$ corresponds to the $3\sigma$ coverage factor.""",

    "IAP-RQ-020": r"""
  The pseudorange residual for satellite $k$ is:
  \begin{equation}
    r_k^{\rho} = z_k^{\rho} - \bigl(\|\mathbf{p}_r - \mathbf{p}_k\| + \delta t\bigr),
    \label{eq:pseudorange}
  \end{equation}
  and the Doppler residual is:
  \begin{equation}
    r_k^{d} = z_k^{d} - \bigl(\mathbf{e}_k^\top(\mathbf{v}_r - \mathbf{v}_k) + \dot{\delta t}\bigr),
    \label{eq:doppler}
  \end{equation}
  where $\mathbf{e}_k = (\mathbf{p}_r - \mathbf{p}_k)/\|\cdot\|$ is the unit line-of-sight vector.""",

    "IAP-RQ-040": r"""
  The LiDAR ICP condition number is computed from the Hessian block:
  \begin{equation}
    \kappa = \frac{\sigma_{\max}(H)}{\sigma_{\min}(H)}, \qquad
    \gamma_{\mathrm{lidar}} = \min\!\left(\sqrt{\kappa / \kappa_{\mathrm{th}}},\, \gamma_{\max}\right),
    \label{eq:icp_cond}
  \end{equation}
  and the LiDAR noise model is inflated by $\gamma_{\mathrm{lidar}}^{2}$ when $\kappa > \kappa_{\mathrm{th}}$.""",

    "IAP-RQ-100": r"""
  Trunk cylinders are fit using the Kasa circle estimator on a 2-D grid cluster.
  The TDOP for the detected trunk constellation is:
  \begin{equation}
    \mathrm{TDOP} = \sqrt{\mathrm{tr}\bigl((G^\top G)^{-1}\bigr)},
    \quad G_{k,:} = \mathbf{e}_{k,xy}^\top,
    \label{eq:tdop}
  \end{equation}
  where $\mathbf{e}_{k,xy}$ is the bearing to trunk $k$ in the horizontal plane.""",

    "IAP-RQ-200": r"""
  The protection level and alert limit are:
  \begin{equation}
    \mathrm{PL} = K_{\mathrm{pl}} \sqrt{\lambda_{\max}(\Sigma_p)}, \qquad
    \mathrm{AL} = \max\!\bigl(a_{\min},\, a_s \cdot d_{\mathrm{obs}} - r_{\mathrm{UAV}}\bigr),
    \label{eq:PL_AL}
  \end{equation}
  and the integrity margin is $\mathrm{IM} = \mathrm{AL} - \mathrm{PL}$.  The system is
  \emph{safe} when $\mathrm{IM} > 0$.""",

    "IAP-RQ-220": r"""
  Per-satellite NIS gating (RAIM-ish):
  \begin{equation}
    \mathrm{NIS}_k = r_k^2 / \sigma_k^2 \overset{H_0}{\sim} \chi^2(1).
    \label{eq:nis}
  \end{equation}
  Satellite $k$ is excluded when $\mathrm{NIS}_k > \chi^2_{1,\alpha}$ with $\alpha=0.01$.""",

    "IAP-RQ-320": r"""
  The isotropic covariance growth model predicts:
  \begin{equation}
    \sigma_{\mathrm{pred}}(t+\Delta t) = \sqrt{\sigma_{\mathrm{pred}}(t)^2 + \sigma_g^2 \Delta t},
    \qquad
    \mathrm{PL}_{\mathrm{pred}}(t) = K_{\mathrm{pl}} \cdot \sigma_{\mathrm{pred}}(t).
    \label{eq:sigma_grow}
  \end{equation}""",

    "IAP-RQ-400": r"""
  The integrity-aware planning cost is:
  \begin{equation}
    J(\tau) = w_I \sum_{k} \bigl[\max(0,\, \mathrm{PL}_{\mathrm{pred},k} - \mathrm{AL})\bigr]^2
              + w_m \|\mathbf{p}_{N} - \mathbf{p}_{\mathrm{goal}}\|
              + w_s \sum_{k} \|\Delta \mathbf{v}_k\|,
    \label{eq:cost}
  \end{equation}
  where $w_I$ is boosted by a factor of $5$ in \textsc{search} mode.""",
}


# ---------------------------------------------------------------------------
# TikZ flowchart placeholder
# ---------------------------------------------------------------------------

TIKZ_FLOW = r"""\begin{tikzpicture}[
    node distance=1.0cm and 1.6cm,
    block/.style={rectangle, draw, rounded corners, text width=3.5cm,
                  align=center, minimum height=0.7cm, font=\small},
    arrow/.style={->, thick}
  ]
  % ---- Sensing column ----
  \node[block] (lidar)  {LiDAR\\(raw points)};
  \node[block, below=of lidar] (gnss) {GNSS\\(pseudorange / Doppler)};
  \node[block, below=of gnss]  (imu)  {IMU};

  % ---- Processing ----
  \node[block, right=2.0cm of lidar]  (icp)  {ICP odometry\\(IAP-RQ-040)};
  \node[block, right=2.0cm of gnss]   (fgo)  {FGO / smoother\\(IAP-RQ-010/020)};
  \node[block, right=2.0cm of imu]    (trunk){Trunk detector\\(IAP-RQ-100)};

  % ---- Integrity ----
  \node[block, right=2.0cm of fgo]    (integ){Integrity monitor\\(IAP-RQ-200)};

  % ---- Planning ----
  \node[block, right=2.0cm of integ]  (pred) {PL predictor\\(IAP-RQ-320)};
  \node[block, below=of pred]         (plan) {Integrity planner\\(IAP-RQ-400)};

  % ---- Arrows ----
  \draw[arrow] (lidar) -- (icp);
  \draw[arrow] (gnss)  -- (fgo);
  \draw[arrow] (imu)   -- (fgo);
  \draw[arrow] (icp)   -- (fgo);
  \draw[arrow] (trunk) -- (integ);
  \draw[arrow] (fgo)   -- (integ);
  \draw[arrow] (integ) -- (pred);
  \draw[arrow] (pred)  -- (plan);
  \draw[arrow] (plan)  -- ++(0,-1.5) node[below, font=\small]{Execution target};
\end{tikzpicture}"""


# ---------------------------------------------------------------------------
# Main generator
# ---------------------------------------------------------------------------

def generate(traceability_path: str, output_path: str, figure_path: str):
    rows = parse_traceability(traceability_path)

    # Group rows into "known" sections
    section_order = [
        ("State Estimation", [r for r in rows if any(r["req_id"].startswith(p) for p in
                               ["IAP-RQ-010", "IAP-RQ-015", "IAP-RQ-020", "IAP-RQ-040",
                                "IAP-RQ-050"])]),
        ("Environmental Perception", [r for r in rows if any(r["req_id"].startswith(p) for p in
                                      ["IAP-RQ-100", "IAP-RQ-110", "IAP-RQ-120"])]),
        ("Integrity Monitoring", [r for r in rows if any(r["req_id"].startswith(p) for p in
                                   ["IAP-RQ-200", "IAP-RQ-210", "IAP-RQ-220", "IAP-RQ-230",
                                    "IAP-RQ-240"])]),
        ("Trajectory Prediction", [r for r in rows if any(r["req_id"].startswith(p) for p in
                                    ["IAP-RQ-300", "IAP-RQ-310", "IAP-RQ-320"])]),
        ("Integrity-Aware Planning", [r for r in rows if any(r["req_id"].startswith(p) for p in
                                      ["IAP-RQ-400", "IAP-RQ-410"])]),
        ("Experiments \\& Metrics", [r for r in rows if any(r["req_id"].startswith(p) for p in
                                      ["IAP-RQ-500", "IAP-RQ-510"])]),
    ]

    lines = []

    # --- Preamble ---
    lines.append(r"""\documentclass[journal]{IEEEtran}
\usepackage{amsmath,amssymb}
\usepackage{graphicx}
\usepackage{tikz}
\usetikzlibrary{shapes,arrows,positioning}
\usepackage{hyperref}
\usepackage{booktabs}

% Auto-generated by tools/gen_methodology.py — DO NOT EDIT MANUALLY
% Generated: """ + datetime.utcnow().strftime("%Y-%m-%d %H:%M UTC") + r"""

\begin{document}

%% =========================================================================
\section{Methodology}
\label{sec:methodology}
%% =========================================================================

This section describes the proposed Integrity-Aware Planning (IAP) framework.
The system integrates LiDAR-inertial state estimation with GNSS tightly-coupled
fusion, online integrity monitoring, and receding-horizon planning whose cost
function penalises Protection Level (PL) violations.

%% -------------------------------------------------------------------------
\subsection{System Overview}
\label{sec:overview}
%% -------------------------------------------------------------------------

Figure~\ref{fig:system_flow} shows the overall data flow.

\begin{figure}[ht]
  \centering
""")

    # Figure reference
    if figure_path.endswith(".pdf"):
        lines.append(r"  \includegraphics[width=\columnwidth]{" + figure_path + "}")
    else:
        lines.append(r"  % TikZ inline flowchart (remove if using external figure)")
        lines.append(TIKZ_FLOW)

    lines.append(r"""  \caption{IAP system flowchart. Grey arrows show data flow; coloured blocks
           correspond to the modules described in the subsections below.}
  \label{fig:system_flow}
\end{figure}

""")

    # --- Sections ---
    for (sec_title, sec_rows) in section_order:
        if not sec_rows:
            continue
        lines.append(r"%% -------------------------------------------------------------------------")
        lines.append(r"\subsection{" + sec_title + "}")
        lines.append(r"\label{sec:" + sec_title.lower().replace(" ", "_").replace("\\&", "and") + "}")
        lines.append(r"%% -------------------------------------------------------------------------")
        lines.append("")

        for row in sec_rows:
            rid    = row["req_id"]
            desc   = escape_latex(row["desc"])
            impl   = escape_latex(row["impl"])
            test   = escape_latex(row["test"])
            met    = escape_latex(row["metrics"])
            status = escape_latex(row["status"])

            lines.append(r"\subsubsection{" + escape_latex(rid) + ": " + desc + "}")
            lines.append(r"\label{ssec:" + rid.lower().replace("-", "_") + "}")
            lines.append("")
            lines.append(r"\textbf{Implementation:} \texttt{" + impl + r"}.")
            lines.append("")

            # Insert formula if we have one
            formula = _FORMULA_MAP.get(rid, "")
            if formula:
                lines.append(formula.strip())
                lines.append("")

            lines.append(r"\textbf{Verification:} " + test + r".")
            lines.append(r"\textbf{Logged metrics:} \texttt{" + met + r"}.")
            lines.append(r"\textbf{Status:} " + status + r".")
            lines.append("")

    # --- Traceability table ---
    lines.append(r"""%% -------------------------------------------------------------------------
\subsection{Traceability Summary}
\label{sec:traceability}
%% -------------------------------------------------------------------------

Table~\ref{tab:trace} summarises the requirement-to-implementation mapping.

\begin{table*}[ht]
\caption{Requirement traceability matrix (auto-generated from \texttt{docs/TRACEABILITY.md})}
\label{tab:trace}
\centering
\small
\begin{tabular}{@{}lp{4.5cm}p{5.5cm}p{2.5cm}l@{}}
\toprule
Req ID & Description & Implementation & Metrics & Status \\
\midrule""")

    for row in rows:
        rid    = escape_latex(row["req_id"])
        desc   = escape_latex(row["desc"])
        impl   = escape_latex(row["impl"])
        met    = escape_latex(row["metrics"])
        status = escape_latex(row["status"])
        lines.append(f"\\texttt{{{rid}}} & {desc} & \\texttt{{{impl}}} & \\texttt{{{met}}} & {status} \\\\")

    lines.append(r"""\bottomrule
\end{tabular}
\end{table*}

\end{document}
""")

    # Write output
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print(f"[gen_methodology] Wrote {output_path}  ({len(rows)} rows from traceability)")


# ---------------------------------------------------------------------------
# TikZ figure helper
# ---------------------------------------------------------------------------

def generate_tikz_figure(output_path: str):
    """Write a standalone TikZ flowchart that can be compiled to PDF."""
    content = r"""\documentclass[tikz,border=4pt]{standalone}
\usepackage{tikz}
\usetikzlibrary{shapes,arrows,positioning}
\begin{document}
""" + TIKZ_FLOW + r"""
\end{document}
"""
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"[gen_methodology] Wrote TikZ figure source {output_path}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="IAP-RQ-900: Generate methodology.tex")
    parser.add_argument("--traceability", default="docs/TRACEABILITY.md")
    parser.add_argument("--output",       default="docs/methodology/methodology.tex")
    parser.add_argument("--figure",       default="docs/figures/system_flow.pdf")
    args = parser.parse_args()

    # Also generate the TikZ source
    tikz_src = args.figure.replace(".pdf", ".tex")
    generate_tikz_figure(tikz_src)

    generate(args.traceability, args.output, args.figure)


if __name__ == "__main__":
    main()
