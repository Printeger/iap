# Exact post-optimization observable; ECEF state and SI units.
int32 sat_id
uint8 constellation
float64 pr_meas
float64 dop_meas
float64 pr_sigma
float64 dop_sigma
float64[3] sat_pos
float64[3] sat_vel
float64 tgd
float64 svddt
float64 elevation
float64 azimuth
float64 kappa
float64 pr_residual
float64 nis_pr
float64 nis_dop
bool excluded
