# ICRA-045 verification summary

IAP-RQ-423 closes only the G0C analyzer lexical-output-alias boundary. The
expanded absolute request must now equal canonical resolution after symlink
screening and before root/name/existence validation. Both output roles reject
live `..` detours before analysis and before filesystem mutation rather than
silently canonicalizing them.

The direct red test reproduced exit 0 and target creation for both reviewed
aliases. Green coverage proves exit 2, zero `analyze()` calls and no target,
intermediate directory or other output. A canonical relative analysis name and
canonical absolute draft name still succeed on a fresh valid 15-run synthetic
bundle. Every ICRA-044 adversarial inventory/output/raw-hash test remains
enabled.

Verification:

- focused analyzer 24/24, protocol 6/6, runner 14/14, launch contract 6/6 and
  launch golden 16/16 pass (66/66);
- the one final repository Python discovery passes 405/405;
- Python syntax, fatal-only flake8, JSON validation and `git diff --check`
  pass;
- all 3,829 files across 12 retained ICRA-042 build/install trees preserve the
  exact before/after SHA-256
  `6836841bc7ee74594ff80926bfd67c8531ea2d26076b27406cb9aeea3d784d34`;
- protected fixture and PDF hashes remain exact.

No GPU preflight, ROS, launch, calibration, CTest/retained binary, bag/RViz,
smoke, benchmark, threshold/registry/application, G0D or P5 ran. Result is
`P4_G0C_LIVE_ARTIFACT_PROTOCOL_READY_FOR_REVIEW`, never G0C PASS.
