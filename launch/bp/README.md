# Pre-refactor launch backup

This directory is the source-only transition backup of the root launch
entrypoints that existed before the canonical launch refactor. Do not develop
new behavior here, and do not copy or link these files back into the launch
root.

Historical source tests that inspect these implementations must address `bp/`
explicitly. The directory remains installed only so frozen scripts can resolve
their historical launch basenames; none of its files is a canonical entrypoint
for new development. After all four canonical launches pass their runtime
contracts and frozen references are migrated, the backup can move to `legacy/`
or be removed.
