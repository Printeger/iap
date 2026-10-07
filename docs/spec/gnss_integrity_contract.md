# Current GNSS constellation geometry

The Current Monitor consumes actual optimized pseudorange residuals. Its
constellation authority is `SatObs.constellation`, not a numeric satellite-ID
range. GPS/GAL/BDS/GLO map to the existing fault-prior IDs 0/1/2/3. Unknown
identities are invalid; an enabled but unused system has no clock column.

`GnssAraimLinearizedInput.G` contains ENU position columns followed by one
unit indicator column per used constellation in ascending ID order. Its
metadata and indicator columns must agree. Old mixed-system four-column
inputs are rejected by the production evaluator. GPS-only inputs remain
four columns. `GnssAraimResult.S0` is the full dynamic covariance in that
same ordering, with its upper-left 3x3 block representing position.

Each single-satellite and existing whole-constellation fault hypothesis
rebuilds its remaining design with only remaining clocks. Full and subset
solutions compare three position components; separation covariance is the
subset position block minus the full position block. Clock states of
different dimensions are never subtracted. No clock prior, epsilon, rank
threshold, fault probability or integrity allocation is changed.

Nominal solution validity is distinct from fault-model qualification.
Remaining position degeneracy is preserved in `SubsetSolution.valid`,
`degenerate`, `failure_reason`, and the existing 1e9 PL rejection sentinel.
Removing all observations stays degenerate; dropping an absent clock does
not make an unobservable position observable. The Monitor's existing source
eligibility and fused-motion authority remain unchanged.

The shared position/clock design seam is `gnss/clock_geometry.hpp`. The
frontend uses its separate Vector2 bias/drift graph states; this integrity
geometry marginalizes pseudorange bias only. Advisory raw geometry and
query FIM are still being migrated and do not gain real-data qualification
from this Monitor change. Frozen covariance time, uncertainty identities,
actual matching residuals and independent calibration remain prerequisites.

Regression evidence uses the real evaluator: GPS+BDS augmented weighted
inverse, constant clock shifts, whole-system removal, last-satellite removal,
unknown/legacy identity rejection, and Monitor constellation counts. Existing
GPS golden and serial/parallel checks retain their original numerical gates.

The deprecated `predict_geometry` proxy delegates to this same fault model,
including configured whole-constellation hypotheses. Its GPS-only default
therefore retains nominal validity but returns the degenerate 1e9 bound,
instead of its former single-fault-only finite value. No production planner
caller uses this deprecated proxy; its regression asserts the changed meaning.
