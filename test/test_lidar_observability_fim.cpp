#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <iap/predictor/lidar_observability_fim.hpp>

namespace {

std::filesystem::path predictor_artifact_dir() {
  std::filesystem::path path(IAP_SOURCE_ROOT);
  path /= "docs/dev_predictor/predictor_isolated_test_coverage_artifacts";
  std::filesystem::create_directories(path);
  return path;
}

std::string csv_escape(const std::string& value) {
  bool needs_quotes = false;
  for (const char c : value) {
    if (c == ',' || c == '"' || c == '\n' || c == '\r') {
      needs_quotes = true;
      break;
    }
  }
  if (!needs_quotes) {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value) {
    escaped += c;
    if (c == '"') {
      escaped += '"';
    }
  }
  escaped += '"';
  return escaped;
}

iap::CurrentIntegrityState make_current(const double tdop = 2.0,
                                        const int n_trunks = 4,
                                        const int excluded = 0) {
  iap::CurrentIntegrityState current;
  current.valid = true;
  current.hpl = 5.0;
  current.vpl = 6.0;
  current.hal = 30.0;
  current.val = 20.0;
  current.im = 14.0;
  current.tdop = tdop;
  current.n_trunks_observed = n_trunks;
  for (int i = 0; i < excluded; ++i) {
    current.excluded_trunk_ids.push_back(100 + i);
  }
  return current;
}

iap::LidarObservabilityFim make_estimator() {
  iap::LidarObservabilityFim::Params params;
  params.search_radius_m = 8.0;
  params.min_points = 12;
  params.good_points = 60;
  params.sigma_lidar_m = 0.5;
  params.condition_ref = 20.0;
  params.condition_max = 1.0e8;
  params.tdop_ref = 2.0;
  params.tdop_max = 20.0;
  return iap::LidarObservabilityFim(params);
}

std::vector<Eigen::Vector3d> rich_cloud() {
  std::vector<Eigen::Vector3d> points;
  for (int ix = -3; ix <= 3; ++ix) {
    for (int iy = -3; iy <= 3; ++iy) {
      for (int iz = -2; iz <= 2; ++iz) {
        if (ix == 0 && iy == 0 && iz == 0) {
          continue;
        }
        points.emplace_back(0.7 * ix, 0.6 * iy, 0.5 * iz);
      }
    }
  }
  return points;
}

std::vector<Eigen::Vector3d> line_cloud() {
  std::vector<Eigen::Vector3d> points;
  for (int i = -30; i <= 30; ++i) {
    if (i == 0) {
      continue;
    }
    points.emplace_back(0.1 * i, 0.0, 0.0);
  }
  return points;
}

std::vector<Eigen::Vector3d> plane_grid(const int half_width,
                                        const double spacing,
                                        const Eigen::Vector3d& offset =
                                            Eigen::Vector3d::Zero()) {
  std::vector<Eigen::Vector3d> points;
  for (int ix = -half_width; ix <= half_width; ++ix) {
    for (int iy = -half_width; iy <= half_width; ++iy) {
      points.emplace_back(offset.x() + spacing * ix,
                          offset.y() + spacing * iy,
                          offset.z());
    }
  }
  return points;
}

int count_primitives_near(
    const std::vector<iap::LidarFimPrimitive>& primitives,
    const Eigen::Vector3d& center,
    const double radius_m) {
  int count = 0;
  const double radius2 = radius_m * radius_m;
  for (const auto& primitive : primitives) {
    if ((primitive.center_w - center).squaredNorm() <= radius2) {
      ++count;
    }
  }
  return count;
}

std::vector<iap::LidarFimPrimitive> axis_primitives(
    const std::vector<Eigen::Vector3d>& normals,
    const int repeats_per_normal) {
  std::vector<iap::LidarFimPrimitive> primitives;
  for (const auto& normal : normals) {
    for (int i = 0; i < repeats_per_normal; ++i) {
      iap::LidarFimPrimitive p;
      p.center_w = 0.25 * static_cast<double>(i - repeats_per_normal / 2) *
                   Eigen::Vector3d::Ones();
      p.normal_w = normal;
      p.weight = 1.0;
      p.normal_confidence = 1.0;
      p.support_count = 8;
      primitives.push_back(p);
    }
  }
  return primitives;
}

std::vector<iap::LidarFimPrimitive> corridor_primitives(
    const bool one_sided) {
  std::vector<iap::LidarFimPrimitive> primitives;
  const std::vector<double> wall_y = one_sided ? std::vector<double>{2.0}
                                               : std::vector<double>{-2.0, 2.0};
  for (const double y : wall_y) {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitY();
    if (y > 0.0) {
      normal = -normal;
    }
    for (int ix = -8; ix <= 8; ++ix) {
      for (int iz = 0; iz <= 2; ++iz) {
        iap::LidarFimPrimitive p;
        p.center_w =
            Eigen::Vector3d(0.5 * static_cast<double>(ix), y, 0.5 * iz);
        p.normal_w = normal;
        p.weight = 1.0;
        p.normal_confidence = 1.0;
        p.support_count = 8;
        primitives.push_back(p);
      }
    }
  }
  return primitives;
}

Eigen::Vector3d sorted_eigenvalues(const Eigen::Matrix3d& matrix) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(
      0.5 * (matrix + matrix.transpose()), Eigen::EigenvaluesOnly);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  return eig.eigenvalues();
}

}  // namespace

TEST(LidarObservabilityFimTest, RichCloudProducesValidInformation) {
  const auto points = rich_cloud();
  const auto result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &points, make_current());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.delta_lambda.allFinite());
  EXPECT_GT(result.delta_lambda.trace(), 0.0);
  EXPECT_TRUE(std::isfinite(result.tdop_proxy));
  EXPECT_GT(result.lidar_alpha, 0.0);
  EXPECT_EQ(result.fallback_reason, "");
}

TEST(LidarObservabilityFimTest, TooFewPointsFallbackIsExplicit) {
  std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
  };

  const auto result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &points, make_current());

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.lidar_alpha, 0.0);
  EXPECT_EQ(result.fallback_reason, "too_few_points");
  EXPECT_TRUE(std::isfinite(result.tdop_proxy));
  EXPECT_TRUE(std::isfinite(result.condition));
}

TEST(LidarObservabilityFimTest, DegenerateGeometryLowersAlpha) {
  const auto rich = rich_cloud();
  const auto line = line_cloud();
  const auto rich_result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &rich, make_current());
  const auto line_result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &line, make_current());

  ASSERT_TRUE(rich_result.valid);
  EXPECT_LT(line_result.lidar_alpha, rich_result.lidar_alpha);
  EXPECT_FALSE(line_result.valid);
  EXPECT_EQ(line_result.fallback_reason, "degenerate_geometry");
  EXPECT_TRUE(std::isfinite(line_result.tdop_proxy));
  EXPECT_TRUE(std::isfinite(line_result.condition));
}

TEST(LidarObservabilityFimTest, PoorCurrentTdopAndExclusionsDownweightAlpha) {
  const auto points = rich_cloud();
  const auto nominal =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &points, make_current(2.0, 4, 0));
  const auto poor =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), &points, make_current(15.0, 4, 2));

  ASSERT_TRUE(nominal.valid);
  ASSERT_TRUE(poor.valid);
  EXPECT_LT(poor.lidar_alpha, nominal.lidar_alpha);
}

TEST(LidarObservabilityFimTest, MissingCloudFallbackIsFinite) {
  const auto result =
      make_estimator().evaluate(Eigen::Vector3d::Zero(), nullptr, make_current());

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.lidar_alpha, 0.0);
  EXPECT_EQ(result.fallback_reason, "missing_lidar_map");
  EXPECT_TRUE(std::isfinite(result.tdop_proxy));
  EXPECT_TRUE(std::isfinite(result.condition));
}

TEST(LidarObservabilityFimTest, InvalidParamsFallbackIsFinite) {
  iap::LidarObservabilityFim::Params params;
  params.search_radius_m = 0.0;
  params.min_points = 12;
  params.good_points = 80;
  iap::LidarObservabilityFim estimator(params);
  const auto points = rich_cloud();

  const auto result =
      estimator.evaluate(Eigen::Vector3d::Zero(), &points, make_current());

  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(result.lidar_alpha, 0.0);
  EXPECT_EQ(result.fallback_reason, "invalid_lidar_params");
  EXPECT_TRUE(std::isfinite(result.tdop_proxy));
  EXPECT_TRUE(std::isfinite(result.condition));
}

TEST(LidarObservabilityFimTest, AdvisoryFimReflectsNormalAnisotropy) {
  iap::LidarObservabilityFim::Params params;
  params.fim_radius_m = 10.0;
  params.fim_min_voxels = 6;
  params.fim_range_sigma_base = 1.0;
  params.fim_condition_max = 1.0e8;
  params.fim_weight_scale = 1.0;
  iap::LidarObservabilityFim estimator(params);

  std::vector<iap::LidarFimPrimitive> primitives;
  for (int i = 0; i < 24; ++i) {
    iap::LidarFimPrimitive p;
    p.center_w = Eigen::Vector3d(0.1 * i, 0.0, 0.0);
    p.normal_w = Eigen::Vector3d::UnitX();
    primitives.push_back(p);
  }
  for (int i = 0; i < 8; ++i) {
    iap::LidarFimPrimitive p;
    p.center_w = Eigen::Vector3d(0.0, 0.1 * i, 0.0);
    p.normal_w = Eigen::Vector3d::UnitY();
    primitives.push_back(p);
  }
  for (int i = 0; i < 8; ++i) {
    iap::LidarFimPrimitive p;
    p.center_w = Eigen::Vector3d(0.0, 0.0, 0.1 * i);
    p.normal_w = Eigen::Vector3d::UnitZ();
    primitives.push_back(p);
  }

  const auto result = estimator.evaluate_advisory_fim(
      Eigen::Vector3d::Zero(), &primitives, make_current());

  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.lambda(0, 0), result.lambda(1, 1));
  EXPECT_GT(result.lambda(0, 0), result.lambda(2, 2));
  EXPECT_GT(result.trace, 0.0);
  EXPECT_EQ(result.n_valid_normals, static_cast<int>(primitives.size()));
}

TEST(LidarObservabilityFimTest, AdvisoryFimIndexMatchesVectorScan) {
  iap::LidarObservabilityFim::Params params;
  params.fim_radius_m = 6.0;
  params.fim_min_voxels = 6;
  params.fim_range_sigma_base = 1.0;
  params.fim_condition_max = 1.0e8;
  params.fim_weight_scale = 1.0;
  iap::LidarObservabilityFim estimator(params);
  auto primitives =
      std::make_shared<std::vector<iap::LidarFimPrimitive>>(
          axis_primitives({Eigen::Vector3d::UnitX(),
                           Eigen::Vector3d::UnitY(),
                           Eigen::Vector3d::UnitZ()},
                          24));
  const auto index =
      iap::LidarFimPrimitiveIndex::build(primitives, params.fim_radius_m);
  const Eigen::Vector3d query(0.25, -0.2, 0.1);

  const auto vector_result = estimator.evaluate_advisory_fim(
      query, primitives.get(), make_current());
  const auto indexed_result = estimator.evaluate_advisory_fim(
      query, index.get(), make_current());

  ASSERT_TRUE(vector_result.valid);
  ASSERT_TRUE(indexed_result.valid);
  EXPECT_EQ(indexed_result.n_primitives, vector_result.n_primitives);
  EXPECT_EQ(indexed_result.n_valid_normals, vector_result.n_valid_normals);
  EXPECT_TRUE(indexed_result.lambda.isApprox(vector_result.lambda, 1.0e-12));
  EXPECT_NEAR(indexed_result.condition, vector_result.condition, 1.0e-9);
}

TEST(LidarObservabilityFimTest, DegenerateAdvisoryFimIsRegularizedValid) {
  iap::LidarObservabilityFim::Params params;
  params.fim_radius_m = 8.0;
  params.fim_min_voxels = 6;
  params.fim_range_sigma_base = 1.0;
  params.fim_condition_max = 10.0;
  iap::LidarObservabilityFim estimator(params);
  const auto corridor = corridor_primitives(false);

  const auto result = estimator.evaluate_advisory_fim(
      Eigen::Vector3d::Zero(), &corridor, make_current());

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.regularized);
  EXPECT_EQ(result.fallback_reason, "");
  EXPECT_GT(result.n_valid_normals, params.fim_min_voxels);
  EXPECT_GT(result.condition, params.fim_condition_max);
}

TEST(LidarObservabilityFimTest, AdvisoryFimRequiresNormals) {
  iap::LidarObservabilityFim estimator;
  const auto result = estimator.evaluate_advisory_fim(
      Eigen::Vector3d::Zero(), nullptr, make_current());

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.fallback_reason, "missing_lidar_normals");
  EXPECT_DOUBLE_EQ(result.lambda.trace(), 0.0);
}

TEST(LidarObservabilityFimTest, CorridorGeometryWeakensAlongCorridorInformation) {
  iap::LidarObservabilityFim::Params params;
  params.fim_radius_m = 8.0;
  params.fim_min_voxels = 6;
  params.fim_range_sigma_base = 1.0;
  params.fim_condition_max = 1.0e13;
  params.fim_weight_scale = 1.0;
  iap::LidarObservabilityFim estimator(params);

  const auto rich = axis_primitives(
      {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
       Eigen::Vector3d::UnitZ()},
      16);
  const auto corridor = corridor_primitives(false);
  const auto one_sided = corridor_primitives(true);

  const auto rich_result = estimator.evaluate_advisory_fim(
      Eigen::Vector3d::Zero(), &rich, make_current());
  const auto corridor_result = estimator.evaluate_advisory_fim(
      Eigen::Vector3d::Zero(), &corridor, make_current());
  const auto one_sided_result = estimator.evaluate_advisory_fim(
      Eigen::Vector3d::Zero(), &one_sided, make_current());

  ASSERT_TRUE(rich_result.valid);
  ASSERT_TRUE(corridor_result.valid);
  ASSERT_TRUE(one_sided_result.valid);

  std::ofstream csv(predictor_artifact_dir() /
                    "lidar_corridor_degeneracy.csv");
  csv << "case_id,n_primitives,lambda_xx,lambda_yy,lambda_zz,min_eig,"
      << "mid_eig,max_eig,condition,tdop,normal_diversity_score,"
      << "along_corridor_sigma,degeneracy_score,allowed_for_fusion,"
      << "fallback_reason\n";

  auto write_row = [&](const std::string& case_id,
                       const iap::LidarAdvisoryFimResult& result) {
    const Eigen::Vector3d eig = sorted_eigenvalues(result.lambda);
    const double lambda_xx = result.lambda(0, 0);
    const double lambda_yy = result.lambda(1, 1);
    const double lambda_zz = result.lambda(2, 2);
    const double trace = std::max(result.lambda.trace(), 1.0e-12);
    const double normal_diversity = eig(0) / std::max(eig(2), 1.0e-12);
    const double along_sigma = 1.0 / std::sqrt(std::max(lambda_xx, 1.0e-9));
    const double degeneracy_score =
        1.0 - std::clamp(normal_diversity, 0.0, 1.0);
    const bool allowed =
        result.valid && result.fallback_reason.empty() &&
        std::isfinite(result.condition) && result.condition < 1.0e8 &&
        normal_diversity > 1.0e-3;
    csv << csv_escape(case_id) << ','
        << result.n_primitives << ','
        << lambda_xx << ','
        << lambda_yy << ','
        << lambda_zz << ','
        << eig(0) << ','
        << eig(1) << ','
        << eig(2) << ','
        << result.condition << ','
        << result.trace << ','
        << normal_diversity << ','
        << along_sigma << ','
        << degeneracy_score << ','
        << (allowed ? 1 : 0) << ','
        << csv_escape(result.fallback_reason.empty() ? "none"
                                                     : result.fallback_reason)
        << '\n';
    (void)trace;
  };

  write_row("rich", rich_result);
  write_row("corridor", corridor_result);
  write_row("one_sided", one_sided_result);

  EXPECT_LT(corridor_result.lambda(0, 0), 0.1 * rich_result.lambda(0, 0));
  EXPECT_GT(corridor_result.lambda(1, 1), corridor_result.lambda(0, 0));
  EXPECT_GT(corridor_result.condition, rich_result.condition);
  EXPECT_GT(one_sided_result.condition, rich_result.condition);
  const double along_sigma =
      1.0 / std::sqrt(std::max(corridor_result.lambda(0, 0), 1.0e-9));
  const double cross_sigma =
      1.0 / std::sqrt(std::max(corridor_result.lambda(1, 1), 1.0e-9));
  EXPECT_GT(along_sigma, cross_sigma);
}

TEST(LidarObservabilityFimTest, PcaRadiusChangesPrimitiveCount) {
  const auto points = plane_grid(3, 0.4);
  iap::LidarFimPrimitiveGenerationParams small_radius;
  small_radius.pca_radius_m = 0.45;
  small_radius.pca_min_support = 5;
  small_radius.pca_voxel_sample_m = 0.1;
  small_radius.pca_max_points = 1000;
  small_radius.pca_max_primitives = 1000;
  iap::LidarFimPrimitiveGenerationParams large_radius = small_radius;
  large_radius.pca_radius_m = 0.75;

  iap::LidarFimPrimitiveGenerationDiagnostics small_diag;
  iap::LidarFimPrimitiveGenerationDiagnostics large_diag;
  const auto small =
      iap::make_lidar_fim_primitives(points, nullptr, small_radius, &small_diag);
  const auto large =
      iap::make_lidar_fim_primitives(points, nullptr, large_radius, &large_diag);

  ASSERT_TRUE(small);
  ASSERT_TRUE(large);
  EXPECT_LT(small->size(), large->size());
  EXPECT_DOUBLE_EQ(small_diag.lidar_pca_radius_m, 0.45);
  EXPECT_DOUBLE_EQ(large_diag.lidar_pca_radius_m, 0.75);
}

TEST(LidarObservabilityFimTest, PcaMinSupportBoundaryIsInclusive) {
  const std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(-1.0, -1.0, 0.0),
      Eigen::Vector3d(-1.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, -1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.4, -0.2, 0.0),
  };
  iap::LidarFimPrimitiveGenerationParams params;
  params.pca_radius_m = 3.0;
  params.pca_max_points = 1;
  params.pca_min_support = 6;
  params.pca_voxel_sample_m = 0.0;
  params.pca_max_primitives = 1;

  const auto boundary =
      iap::make_lidar_fim_primitives(points, nullptr, params, nullptr);
  params.pca_min_support = 7;
  iap::LidarFimPrimitiveGenerationDiagnostics too_high_diag;
  const auto too_high =
      iap::make_lidar_fim_primitives(points, nullptr, params, &too_high_diag);

  ASSERT_TRUE(boundary);
  ASSERT_TRUE(too_high);
  EXPECT_EQ(boundary->size(), 1u);
  EXPECT_TRUE(too_high->empty());
  EXPECT_FALSE(too_high_diag.valid);
  EXPECT_EQ(too_high_diag.fallback_reason, "missing_lidar_normals");
}

TEST(LidarObservabilityFimTest, VoxelSamplingReducesDenseClusterDuplicates) {
  std::vector<Eigen::Vector3d> points;
  const auto dense = plane_grid(6, 0.02, Eigen::Vector3d::Zero());
  const auto sparse = plane_grid(1, 0.2, Eigen::Vector3d(2.0, 0.0, 0.0));
  points.insert(points.end(), dense.begin(), dense.end());
  points.insert(points.end(), sparse.begin(), sparse.end());

  iap::LidarFimPrimitiveGenerationParams no_voxel;
  no_voxel.pca_radius_m = 0.18;
  no_voxel.pca_min_support = 6;
  no_voxel.pca_max_points = 1000;
  no_voxel.pca_max_primitives = 1000;
  no_voxel.pca_voxel_sample_m = 0.0;
  iap::LidarFimPrimitiveGenerationParams voxel = no_voxel;
  voxel.pca_voxel_sample_m = 0.5;

  const auto dense_biased =
      iap::make_lidar_fim_primitives(points, nullptr, no_voxel, nullptr);
  const auto spatial =
      iap::make_lidar_fim_primitives(points, nullptr, voxel, nullptr);

  ASSERT_TRUE(dense_biased);
  ASSERT_TRUE(spatial);
  EXPECT_GT(count_primitives_near(*dense_biased, Eigen::Vector3d::Zero(), 0.3),
            20);
  EXPECT_LE(count_primitives_near(*spatial, Eigen::Vector3d::Zero(), 0.3), 4);
  EXPECT_LT(spatial->size(), dense_biased->size());
}

TEST(LidarObservabilityFimTest, EmptyPrimitiveGenerationReportsMissingNormals) {
  iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
  const auto primitives = iap::make_lidar_fim_primitives(
      std::vector<Eigen::Vector3d>{}, nullptr,
      iap::LidarFimPrimitiveGenerationParams{}, &diagnostics);

  ASSERT_TRUE(primitives);
  EXPECT_TRUE(primitives->empty());
  EXPECT_FALSE(diagnostics.valid);
  EXPECT_EQ(diagnostics.fallback_reason, "missing_lidar_normals");

  iap::LidarObservabilityFim estimator;
  const auto result = estimator.evaluate_advisory_fim(
      Eigen::Vector3d::Zero(), primitives.get(), make_current());
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.fallback_reason, "missing_lidar_normals");
}

TEST(LidarObservabilityFimTest, CloudProvidedNormalsAreUsedBeforePca) {
  const std::vector<Eigen::Vector3d> points = {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0),
      Eigen::Vector3d(0.5, 0.5, 0.0),
  };
  const std::vector<Eigen::Vector3d> normals(
      points.size(), Eigen::Vector3d::UnitX());
  iap::LidarFimPrimitiveGenerationParams params;
  params.pca_radius_m = 0.1;
  params.pca_min_support = 100;
  params.use_cloud_normals_first = true;
  iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;

  const auto primitives =
      iap::make_lidar_fim_primitives(points, &normals, params, &diagnostics);

  ASSERT_TRUE(primitives);
  ASSERT_EQ(primitives->size(), points.size());
  EXPECT_TRUE(diagnostics.valid);
  EXPECT_EQ(diagnostics.lidar_pca_valid_normals,
            static_cast<int>(points.size()));
  EXPECT_EQ(diagnostics.lidar_pca_invalid_normals, 0);
  for (const auto& primitive : *primitives) {
    EXPECT_EQ(primitive.support_count, 1);
    EXPECT_NEAR(primitive.normal_w.dot(Eigen::Vector3d::UnitX()), 1.0, 1.0e-12);
  }
}

TEST(LidarObservabilityFimTest, PrimitiveGenerationParameterCurvesExport) {
  std::ofstream csv(predictor_artifact_dir() /
                    "lidar_primitive_generation_parameters.csv");
  csv << "experiment,param_value,primitive_count,valid,fallback_reason\n";

  const auto radius_points = plane_grid(3, 0.4);
  for (const double radius : {0.45, 0.75}) {
    iap::LidarFimPrimitiveGenerationParams params;
    params.pca_radius_m = radius;
    params.pca_min_support = 5;
    params.pca_voxel_sample_m = 0.1;
    params.pca_max_points = 1000;
    params.pca_max_primitives = 1000;
    iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
    const auto primitives =
        iap::make_lidar_fim_primitives(radius_points, nullptr, params,
                                       &diagnostics);
    ASSERT_TRUE(primitives);
    csv << "pca_radius," << radius << ',' << primitives->size() << ','
        << (diagnostics.valid ? 1 : 0) << ','
        << csv_escape(diagnostics.fallback_reason) << '\n';
  }

  const std::vector<Eigen::Vector3d> support_points = {
      Eigen::Vector3d(-1.0, -1.0, 0.0),
      Eigen::Vector3d(-1.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, -1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.4, -0.2, 0.0),
  };
  for (const int support : {6, 7}) {
    iap::LidarFimPrimitiveGenerationParams params;
    params.pca_radius_m = 3.0;
    params.pca_max_points = 1;
    params.pca_min_support = support;
    params.pca_voxel_sample_m = 0.0;
    params.pca_max_primitives = 1;
    iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
    const auto primitives =
        iap::make_lidar_fim_primitives(support_points, nullptr, params,
                                       &diagnostics);
    ASSERT_TRUE(primitives);
    csv << "pca_min_support," << support << ',' << primitives->size() << ','
        << (diagnostics.valid ? 1 : 0) << ','
        << csv_escape(diagnostics.fallback_reason) << '\n';
  }

  std::vector<Eigen::Vector3d> voxel_points;
  const auto dense = plane_grid(6, 0.02, Eigen::Vector3d::Zero());
  const auto sparse = plane_grid(1, 0.2, Eigen::Vector3d(2.0, 0.0, 0.0));
  voxel_points.insert(voxel_points.end(), dense.begin(), dense.end());
  voxel_points.insert(voxel_points.end(), sparse.begin(), sparse.end());
  for (const double voxel : {0.0, 0.5}) {
    iap::LidarFimPrimitiveGenerationParams params;
    params.pca_radius_m = 0.18;
    params.pca_min_support = 6;
    params.pca_max_points = 1000;
    params.pca_max_primitives = 1000;
    params.pca_voxel_sample_m = voxel;
    iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
    const auto primitives =
        iap::make_lidar_fim_primitives(voxel_points, nullptr, params,
                                       &diagnostics);
    ASSERT_TRUE(primitives);
    csv << "pca_voxel_sample," << voxel << ',' << primitives->size() << ','
        << (diagnostics.valid ? 1 : 0) << ','
        << csv_escape(diagnostics.fallback_reason) << '\n';
  }

  const std::vector<Eigen::Vector3d> normal_points = {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 1.0, 0.0),
      Eigen::Vector3d(1.0, 1.0, 0.0),
      Eigen::Vector3d(0.5, 0.5, 0.0),
  };
  const std::vector<Eigen::Vector3d> normals(
      normal_points.size(), Eigen::Vector3d::UnitX());
  iap::LidarFimPrimitiveGenerationParams normal_params;
  normal_params.pca_radius_m = 0.1;
  normal_params.pca_min_support = 100;
  normal_params.use_cloud_normals_first = true;
  iap::LidarFimPrimitiveGenerationDiagnostics normal_diagnostics;
  const auto normal_primitives = iap::make_lidar_fim_primitives(
      normal_points, &normals, normal_params, &normal_diagnostics);
  ASSERT_TRUE(normal_primitives);
  csv << "cloud_normals_first,1," << normal_primitives->size() << ','
      << (normal_diagnostics.valid ? 1 : 0) << ','
      << csv_escape(normal_diagnostics.fallback_reason) << '\n';

  EXPECT_EQ(normal_primitives->size(), normal_points.size());
}
