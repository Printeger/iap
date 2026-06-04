#!/usr/bin/env python3
"""Export ARAIM PRN-3 fault-injection gtest diagnostics to CSV and plots."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from reference_gnss_wls_pl import SATS, build_G, compute_fault_free_pl


DEFAULT_XML = Path("build/iap/test_results/iap/test_araim.gtest.xml")
DEFAULT_OUTPUT_DIR = Path("src/iap/results/araim_validation")
TEST_CLASS = "iap.AraimFaultInjectionTest"
TEST_NAME = "Prn3ResidualBiasIncreasesSeparationAndDetectsLargeFault"
BIAS_VALUES = [0, 1, 3, 5, 10, 20]
FIELDNAMES = [
    "bias_m",
    "hpl",
    "vpl",
    "gnss_valid",
    "gnss_n_hyp",
    "gnss_n_det",
    "excluded_prn",
    "worst_hypothesis",
    "failure_reason",
    "prn3_d_horiz",
    "prn3_d_vert",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export ARAIM fault-injection diagnostics from gtest XML."
    )
    parser.add_argument(
        "--xml",
        type=Path,
        default=DEFAULT_XML,
        help=f"gtest XML path, default: {DEFAULT_XML}",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"output directory, default: {DEFAULT_OUTPUT_DIR}",
    )
    return parser.parse_args()


def find_fault_injection_properties(xml_path: Path) -> dict[str, str]:
    if not xml_path.exists():
        raise FileNotFoundError(
            f"Missing gtest XML: {xml_path}. Run test_araim before exporting."
        )

    root = ET.parse(xml_path).getroot()
    for testcase in root.iter("testcase"):
        if (
            testcase.get("classname") == TEST_CLASS
            and testcase.get("name") == TEST_NAME
        ):
            props = testcase.find("properties")
            if props is None:
                raise RuntimeError(f"Testcase {TEST_NAME} has no properties.")
            return {
                prop.get("name"): prop.get("value", "")
                for prop in props.findall("property")
                if prop.get("name")
            }
    raise RuntimeError(f"Could not find {TEST_CLASS}.{TEST_NAME} in {xml_path}.")


def prop(props: dict[str, str], name: str, default: str = "") -> str:
    return props.get(name, default)


def build_rows(props: dict[str, str]) -> list[dict[str, str]]:
    rows = []
    for bias in BIAS_VALUES:
        prefix = f"bias_{bias}m"
        excluded_prn3 = int(float(prop(props, f"{prefix}_excluded_prn3", "0")))
        rows.append(
            {
                "bias_m": str(bias),
                "hpl": prop(props, f"{prefix}_HPL"),
                "vpl": prop(props, f"{prefix}_VPL"),
                "gnss_valid": prop(props, f"{prefix}_valid", "1"),
                "gnss_n_hyp": prop(props, f"{prefix}_n_hypotheses"),
                "gnss_n_det": prop(props, f"{prefix}_n_detected"),
                "excluded_prn": "3" if excluded_prn3 else "",
                "worst_hypothesis": prop(props, f"{prefix}_worst_hyp"),
                "failure_reason": prop(props, f"{prefix}_failure_reason", "none"),
                "prn3_d_horiz": prop(props, f"{prefix}_prn3_d_horiz"),
                "prn3_d_vert": prop(props, f"{prefix}_prn3_d_vert"),
            }
        )
    return rows


def write_csv(rows: list[dict[str, str]], output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "gnss_fault_injection.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    return csv_path


def to_float(rows: list[dict[str, str]], field: str) -> list[float]:
    return [float(row[field]) for row in rows]


def python_reference_hpl_vpl() -> tuple[float, float]:
    result = compute_fault_free_pl(build_G(SATS))
    return float(result["HPL"]), float(result["VPL"])


def plot_pl_vs_bias(rows: list[dict[str, str]], output_dir: Path) -> Path:
    path = output_dir / "pl_vs_bias.png"
    bias = to_float(rows, "bias_m")
    hpl = to_float(rows, "hpl")
    vpl = to_float(rows, "vpl")
    ref_hpl, ref_vpl = python_reference_hpl_vpl()

    plt.figure()
    plt.plot(bias, hpl, marker="o", label="HPL")
    plt.plot(bias, vpl, marker="o", label="VPL")
    plt.axhline(
        ref_hpl,
        linestyle="--",
        linewidth=1.2,
        color="tab:blue",
        alpha=0.7,
        label=f"Python ref HPL ({ref_hpl:.3f} m)",
    )
    plt.axhline(
        ref_vpl,
        linestyle="--",
        linewidth=1.2,
        color="tab:orange",
        alpha=0.7,
        label=f"Python ref VPL ({ref_vpl:.3f} m)",
    )
    plt.xlabel("Injected pseudorange bias [m]")
    plt.ylabel("Protection Level [m]")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()
    return path


def plot_detected_fault_vs_bias(rows: list[dict[str, str]], output_dir: Path) -> Path:
    path = output_dir / "detected_fault_vs_bias.png"
    bias = to_float(rows, "bias_m")
    n_det = to_float(rows, "gnss_n_det")

    plt.figure()
    plt.plot(bias, n_det, marker="o")
    plt.xlabel("Injected pseudorange bias [m]")
    plt.ylabel("Detected fault count")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=200)
    plt.close()
    return path


def main() -> None:
    args = parse_args()
    props = find_fault_injection_properties(args.xml)
    rows = build_rows(props)
    csv_path = write_csv(rows, args.output_dir)
    pl_path = plot_pl_vs_bias(rows, args.output_dir)
    det_path = plot_detected_fault_vs_bias(rows, args.output_dir)
    ref_hpl, ref_vpl = python_reference_hpl_vpl()
    print(f"Wrote {csv_path}")
    print(f"Wrote {pl_path}")
    print(f"Wrote {det_path}")
    print(f"Python reference HPL={ref_hpl:.12f}, VPL={ref_vpl:.12f}")


if __name__ == "__main__":
    main()
