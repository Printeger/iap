// IAP-RQ-100: Trunk detection + parameterization
// IAP-RQ-110: Trunk health factor (Baseline-A)
// IAP-RQ-120: TDOP metric

#include <iap/trunk/trunk_detector.hpp>
#include <iap/util/timing_csv.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace iap {

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
namespace {

using Grid2i = std::pair<int, int>;

struct Grid2iHash {
  std::size_t operator()(const Grid2i& g) const noexcept {
    // Cantor pairing with sign handling
    const std::size_t a = static_cast<std::size_t>(g.first  + 65536);
    const std::size_t b = static_cast<std::size_t>(g.second + 65536);
    return (a + b) * (a + b + 1) / 2 + b;
  }
};

}  // namespace

// ---------------------------------------------------------------------------
TrunkDetector::TrunkDetector() : params_(Params{}) {}
TrunkDetector::TrunkDetector(const Params& params) : params_(params) {}

// ---------------------------------------------------------------------------
std::tuple<double, double, double, double>
TrunkDetector::kasa_fit(const std::vector<Eigen::Vector2d>& pts) const {
  if (pts.size() < 3) {
    return {0.0, 0.0, 0.0, 0.0};
  }

  // Centroid
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& p : pts) mean += p;
  mean /= static_cast<double>(pts.size());

  // Build linear system: [u, v, 1] * [A, B, C]^T = u^2 + v^2  (Kasa)
  Eigen::MatrixXd M(pts.size(), 3);
  Eigen::VectorXd rhs(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const double u = pts[i](0) - mean(0);
    const double v = pts[i](1) - mean(1);
    M(i, 0) = u;
    M(i, 1) = v;
    M(i, 2) = 1.0;
    rhs(i)  = u * u + v * v;
  }

  // Least-squares solution via normal equations (fast for small N)
  const Eigen::Vector3d sol = (M.transpose() * M).ldlt().solve(M.transpose() * rhs);
  const double A = sol(0), B = sol(1), C = sol(2);

  // Centre and radius (add back mean)
  const double cx = mean(0) + A * 0.5;
  const double cy = mean(1) + B * 0.5;
  const double r2 = A * A * 0.25 + B * B * 0.25 + C;
  if (r2 <= 0.0) {
    return {cx, cy, 0.0, 0.0};
  }
  const double r = std::sqrt(r2);

  // Inlier fraction
  int inliers = 0;
  for (const auto& p : pts) {
    const double dist = std::abs((p - Eigen::Vector2d(cx, cy)).norm() - r);
    if (dist < params_.fit_tolerance) ++inliers;
  }
  const double inlier_frac = static_cast<double>(inliers) / pts.size();

  return {cx, cy, r, inlier_frac};
}

// ---------------------------------------------------------------------------
TrunkDetectionResult TrunkDetector::detect(const gtsam_points::PointCloud& frame,
                                            double stamp) const {
  const auto t0_trunk = std::chrono::high_resolution_clock::now();
  TrunkDetectionResult result;
  result.stamp   = stamp;
  result.tdop    = params_.tdop_inf;
  result.tdop2   = params_.tdop_inf;
  result.tdop_weighted = params_.tdop_inf;

  if (!frame.points || frame.size() == 0) {
    return result;
  }

  const double res = params_.grid_resolution;

  // ---- 1. Height + range filter → 2-D grid occupancy --------------------
  // Map cell → list of 3D points
  std::unordered_map<Grid2i, std::vector<Eigen::Vector2d>, Grid2iHash> grid;
  // Also track z extent per cell
  std::unordered_map<Grid2i, std::pair<double,double>, Grid2iHash> z_range;

  for (int i = 0; i < static_cast<int>(frame.size()); ++i) {
    const double x = frame.points[i](0);
    const double y = frame.points[i](1);
    const double z = frame.points[i](2);
    if (z < params_.trunk_z_min || z > params_.trunk_z_max) continue;
    const double r2d = std::hypot(x, y);    if (r2d < params_.trunk_range_min || r2d > params_.trunk_range_max) continue;

    const int gx = static_cast<int>(std::floor(x / res));
    const int gy = static_cast<int>(std::floor(y / res));
    const Grid2i key{gx, gy};
    grid[key].emplace_back(x, y);
    auto& zr = z_range[key];
    zr.first  = std::min(zr.first,  z);
    zr.second = std::max(zr.second, z);
  }

  if (grid.empty()) {
    return result;
  }

  // ---- 2. Grid-based BFS clustering (8-connected) -------------------------
  std::unordered_map<Grid2i, bool, Grid2iHash> visited;
  std::vector<std::vector<Grid2i>> clusters;

  const int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  const int dy[] = {-1, -1, -1, 0, 0,  1,  1, 1};

  for (const auto& kv : grid) {
    if (visited.count(kv.first)) continue;
    // BFS from this cell
    std::vector<Grid2i> cluster;
    std::queue<Grid2i> q;
    q.push(kv.first);
    visited[kv.first] = true;
    while (!q.empty()) {
      const Grid2i cell = q.front(); q.pop();
      cluster.push_back(cell);
      for (int d = 0; d < 8; ++d) {
        const Grid2i nb{cell.first + dx[d], cell.second + dy[d]};
        if (!visited.count(nb) && grid.count(nb)) {
          visited[nb] = true;
          q.push(nb);
        }
      }
    }
    clusters.push_back(std::move(cluster));
  }

  // ---- 3. Fit circle per cluster + compute confidence ---------------------
  for (const auto& cluster_cells : clusters) {
    // Collect 2D points
    std::vector<Eigen::Vector2d> pts;
    double zmin = 1e9, zmax = -1e9;
    for (const auto& cell : cluster_cells) {
      for (const auto& pt : grid.at(cell)) pts.push_back(pt);
      const auto& zr = z_range.at(cell);
      zmin = std::min(zmin, zr.first);
      zmax = std::max(zmax, zr.second);
    }

    const int n = static_cast<int>(pts.size());
    if (n < params_.min_cluster_pts || n > params_.max_cluster_pts) continue;

    auto [cx, cy, r, inlier_frac] = kasa_fit(pts);

    // Reject radius out of plausible range
    if (r < params_.radius_min || r > params_.radius_max) continue;

    // Confidence: inlier_fraction * sigmoid(n) * radius_bonus
    const double n_sig = 1.0 / (1.0 + std::exp(-0.1 * (n - 20.0)));  // saturates ~100pts
    const double confidence = std::clamp(inlier_frac * n_sig, 0.0, 1.0);

    if (confidence < params_.min_confidence) continue;

    TrunkObservation obs;
    obs.center_xy    = Eigen::Vector2d(cx, cy);
    obs.radius       = r;
    obs.z_min        = zmin;
    obs.z_max        = zmax;
    obs.num_points   = n;
    obs.inlier_fraction = inlier_frac;
    obs.confidence   = confidence;
    obs.bearing_xy   = obs.center_xy.normalized();

    result.trunks.push_back(obs);
  }

  // ---- 4. Compute TDOP (IAP-RQ-120) --------------------------------------
  compute_tdop(result);

  // ---- 5. Compute azimuth histogram (IAP-RQ-150) -------------------------
  result.azimuth_histogram = compute_azimuth_histogram(
      result.trunks, Eigen::Vector2d::Zero());

  // IAP-RQ-002: timing measurement
  {
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t0_trunk).count();
    timing_csv::append(stamp, "1.3_trunk_detector", elapsed_ms);
  }

  return result;
}

// ---------------------------------------------------------------------------
void TrunkDetector::compute_tdop(TrunkDetectionResult& result) const {
  const int K = static_cast<int>(result.trunks.size());
  result.tdop    = params_.tdop_inf;
  result.tdop2   = params_.tdop_inf;
  result.tdop_weighted = params_.tdop_inf;
  result.lambda_min_H = 0.0;

  if (K < 2) return;

  // Design matrix G: each row = 2D unit bearing to trunk k
  Eigen::MatrixXd G(K, 2);
  for (int k = 0; k < K; ++k) {
    G.row(k) = result.trunks[k].bearing_xy.transpose();
  }

  // Information matrix H = G^T * G  (2×2)
  const Eigen::Matrix2d H = G.transpose() * G;

  // Eigenvalues for TDOP and degeneracy check
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(H);
  const double lambdas_0 = eig.eigenvalues()(0);  // smaller
  const double lambdas_1 = eig.eigenvalues()(1);  // larger
  result.lambda_min_H = lambdas_0;

  if (lambdas_0 < 1e-9) {
    // Degenerate: all trunks in a line → TDOP = inf
    return;
  }

  // TDOP = sqrt(trace(H^{-1})) = sqrt(1/λ0 + 1/λ1)
  result.tdop2 = std::sqrt(1.0 / lambdas_0 + 1.0 / lambdas_1);
  result.tdop  = result.tdop2;  // 2D only for now

  // IAP-RQ-133: Confidence-weighted TDOP
  // W = diag(conf_k^2),  H_w = G^T W G,  TDOP_w = sqrt(trace(H_w^{-1}))
  {
    Eigen::VectorXd w(K);
    for (int k = 0; k < K; ++k) {
      const double c = std::max(result.trunks[k].confidence, 1e-3);
      w(k) = c * c;
    }
    const Eigen::Matrix2d H_w = G.transpose() * w.asDiagonal() * G;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig_w(H_w);
    const double l0 = eig_w.eigenvalues()(0);
    const double l1 = eig_w.eigenvalues()(1);
    if (l0 > 1e-9) {
      result.tdop_weighted = std::sqrt(1.0 / l0 + 1.0 / l1);
    } else {
      result.tdop_weighted = params_.tdop_inf;
    }
  }
}

// ---------------------------------------------------------------------------
double TrunkDetector::health_factor(const TrunkDetectionResult& result) {
  // IAP-RQ-110 Baseline-A: scalar health from {trunk_count, TDOP, avg_confidence}
  const int K = static_cast<int>(result.trunks.size());
  if (K == 0) return 0.0;

  // Avg confidence
  double avg_conf = 0.0;
  for (const auto& t : result.trunks) avg_conf += t.confidence;
  avg_conf /= K;

  // TDOP contribution: good TDOP ≤ 2, bad TDOP ≥ 10
  const double tdop_score = (result.tdop < 1e8)
    ? std::clamp(1.0 - (result.tdop - 2.0) / 8.0, 0.0, 1.0)
    : 0.0;

  // Count score: saturates at K=5
  const double count_score = std::min(K / 5.0, 1.0);

  return std::clamp(avg_conf * 0.4 + tdop_score * 0.4 + count_score * 0.2, 0.0, 1.0);
}

}  // namespace iap
