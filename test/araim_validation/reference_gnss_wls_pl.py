#!/usr/bin/env python3
"""Compatibility wrapper for the canonical ARAIM validation reference script."""

from pathlib import Path
import runpy


if __name__ == "__main__":
    script = (
        Path(__file__).resolve().parents[2]
        / "scripts"
        / "araim_validation"
        / "reference_gnss_wls_pl.py"
    )
    runpy.run_path(str(script), run_name="__main__")
