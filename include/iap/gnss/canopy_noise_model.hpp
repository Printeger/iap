#pragma once
// IAP-RQ-314: Canopy-aware GNSS noise model
//
// Full variance model (checklist §1.1 / §3.3):
//   σ²_eff = σ²_0 + σ²_mp / sin²(θ) + σ²_c · exp(α · κ / sin(θ))
//
// This header is intentionally pure-function / parameter-struct only
// (header-only, no .cpp required) to keep it lightweight.

#include <cmath>
#include <algorithm>

namespace iap {

/// @brief Parameters for the canopy noise model (IAP-RQ-314).
struct CanopyNoiseParams {
  double sigma_0  = 1.0;   ///< floor pseudorange noise [m]          (σ_0)
  double sigma_mp = 0.5;   ///< multipath noise amplitude [m]        (σ_mp)
  double sigma_c  = 5.0;   ///< canopy base noise [m]                (σ_c)
  double alpha    = 2.0;   ///< canopy amplification factor           (α)
};

/**
 * @brief Compute effective pseudorange noise under canopy.
 *
 * Full three-term variance model:
 * @code
 *   σ²_eff = σ²_0 + σ²_mp / sin²(θ) + σ²_c · exp(α · κ / sin(θ))
 *   σ_eff  = sqrt(σ²_eff)
 * @endcode
 *
 * When κ = 0 this reduces to elevation-dependent noise:
 *   σ_eff ≈ sqrt(σ²_0 + σ²_mp / sin²(θ) + σ²_c)
 *
 * @param p         Canopy noise parameters (σ_0, σ_mp, σ_c, α)
 * @param kappa     Canopy density along LOS ∈ [0, 1]  (IAP-RQ-313)
 * @param elevation Satellite elevation angle [rad]
 * @return σ_eff [m]
 */
inline double sigma_eff_canopy(const CanopyNoiseParams& p,
                               double kappa,
                               double elevation) {
  // Clamp sin(elevation) to avoid division by zero (≥ sin(3°) ≈ 0.052).
  const double sin_el  = std::max(std::sin(elevation), 0.052);
  const double kappa_c = std::max(0.0, std::min(kappa, 1.0));

  const double var_floor  = p.sigma_0  * p.sigma_0;
  const double var_mp     = (p.sigma_mp * p.sigma_mp) / (sin_el * sin_el);
  const double var_canopy = (p.sigma_c  * p.sigma_c)
                            * std::exp(p.alpha * kappa_c / sin_el);

  return std::sqrt(var_floor + var_mp + var_canopy);
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
