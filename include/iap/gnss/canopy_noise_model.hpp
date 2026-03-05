#pragma once
// IAP-RQ-314: Canopy-aware GNSS noise model
//
// Talk §3.2: σ²_eff = σ²_c · exp(α · κ / sin θ)
//
// This header is intentionally pure-function / parameter-struct only
// (header-only, no .cpp required) to keep it lightweight.

#include <cmath>
#include <algorithm>

namespace iap {

/// @brief Parameters for the canopy noise model (IAP-RQ-314).
struct CanopyNoiseParams {
  double sigma_c = 5.0;   ///< base GNSS pseudorange noise [m]  (σ_c in talk)
  double alpha   = 2.0;   ///< canopy amplification factor       (α  in talk)
};

/**
 * @brief Compute effective pseudorange noise under canopy.
 *
 * Talk §3.2:
 * @code
 *   σ²_eff = σ²_c · exp(α · κ / sin(θ))
 * @endcode
 *
 * @param p         Canopy noise parameters (σ_c, α)
 * @param kappa     Canopy density along LOS ∈ [0, 1]  (IAP-RQ-313)
 * @param elevation Satellite elevation angle [rad]
 * @return σ_eff [m]  (always ≥ sigma_c)
 */
inline double sigma_eff_canopy(const CanopyNoiseParams& p,
                               double kappa,
                               double elevation) {
  // Clamp elevation to avoid division by zero.
  const double sin_el = std::max(std::sin(elevation), 0.05);  // min ~3 deg
  const double kappa_c = std::max(0.0, std::min(kappa, 1.0));
  const double exponent = p.alpha * kappa_c / sin_el;
  return p.sigma_c * std::exp(0.5 * exponent);  // sqrt(σ²_c * exp(...))
}

/**
 * @brief Shorthand: return 1/σ_eff² (information weight for WLS).
 */
inline double info_weight_canopy(const CanopyNoiseParams& p,
                                 double kappa,
                                 double elevation) {
  const double s = sigma_eff_canopy(p, kappa, elevation);
  return 1.0 / (s * s);
}

}  // namespace iap
