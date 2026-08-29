#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

namespace iap::sim {

enum class ForkArm { kLowRisk, kHighRisk };

inline constexpr int kForkedForestForkCount = 4;

struct ForkedForestConfig {
  double fork_x_min_m = -16.0;
  double fork_length_m = 8.0;
  double low_risk_amplitude_m = 4.0;
  double high_risk_amplitude_m = 2.8;
  double corridor_half_width_m = 1.2;
  double junction_clearance_radius_m = 2.0;
  double start_x_m = -18.0;
  double goal_x_m = 18.0;
  std::uint32_t fork_risk_seed = 21u;
};

struct ForkCorridorMembership {
  bool inside_low = false;
  bool inside_high = false;
  bool inside_buffer = false;
  int low_fork_index = -1;
  int high_fork_index = -1;

  bool insideAny() const {
    return inside_low || inside_high || inside_buffer;
  }
};

inline std::array<int, kForkedForestForkCount> forkLowRiskSigns(
    const ForkedForestConfig& config) {
  std::array<int, kForkedForestForkCount> signs{};
  std::mt19937 random(config.fork_risk_seed);
  for (int& sign : signs) {
    // Use the MT19937 output bit directly so the expanded preset is stable
    // across standard-library implementations. 0 is right/y<0, 1 is left/y>0.
    sign = ((random() >> 31u) & 1u) == 0u ? -1 : 1;
  }
  return signs;
}

inline int forkArmSign(const ForkedForestConfig& config,
                       const int fork_index,
                       const ForkArm arm) {
  const auto signs = forkLowRiskSigns(config);
  const int bounded =
      std::clamp(fork_index, 0, kForkedForestForkCount - 1);
  const int low_sign = signs[static_cast<std::size_t>(bounded)];
  return arm == ForkArm::kLowRisk ? low_sign : -low_sign;
}

inline double forkArmAmplitude(const ForkedForestConfig& config,
                               const ForkArm arm) {
  return arm == ForkArm::kLowRisk ? config.low_risk_amplitude_m
                                  : config.high_risk_amplitude_m;
}

inline double forkArmCenterY(const ForkedForestConfig& config,
                             const int fork_index,
                             const ForkArm arm,
                             const double x_m) {
  constexpr double kPi = 3.141592653589793238462643383279502884;
  const double x0 = config.fork_x_min_m +
                    static_cast<double>(fork_index) * config.fork_length_m;
  const double t = std::clamp((x_m - x0) / config.fork_length_m, 0.0, 1.0);
  const double wave = std::sin(kPi * t);
  return static_cast<double>(forkArmSign(config, fork_index, arm)) *
         forkArmAmplitude(config, arm) * wave * wave;
}

inline double forkArmSlope(const ForkedForestConfig& config,
                           const int fork_index,
                           const ForkArm arm,
                           const double x_m) {
  constexpr double kPi = 3.141592653589793238462643383279502884;
  const double x0 = config.fork_x_min_m +
                    static_cast<double>(fork_index) * config.fork_length_m;
  const double t = std::clamp((x_m - x0) / config.fork_length_m, 0.0, 1.0);
  return static_cast<double>(forkArmSign(config, fork_index, arm)) *
         forkArmAmplitude(config, arm) * kPi / config.fork_length_m *
         std::sin(2.0 * kPi * t);
}

inline double forkArmLength(const ForkedForestConfig& config,
                            const ForkArm arm) {
  constexpr int kSteps = 4096;
  const double dx = config.fork_length_m / static_cast<double>(kSteps);
  double length = 0.0;
  double previous_x = config.fork_x_min_m;
  double previous_y = forkArmCenterY(config, 0, arm, previous_x);
  for (int step = 1; step <= kSteps; ++step) {
    const double x = config.fork_x_min_m + static_cast<double>(step) * dx;
    const double y = forkArmCenterY(config, 0, arm, x);
    length += std::hypot(x - previous_x, y - previous_y);
    previous_x = x;
    previous_y = y;
  }
  return length;
}

inline double pointSegmentDistance(const double px,
                                   const double py,
                                   const double ax,
                                   const double ay,
                                   const double bx,
                                   const double by) {
  const double vx = bx - ax;
  const double vy = by - ay;
  const double denominator = vx * vx + vy * vy;
  const double projection = denominator > 0.0
      ? std::clamp(((px - ax) * vx + (py - ay) * vy) / denominator,
                   0.0, 1.0)
      : 0.0;
  return std::hypot(px - (ax + projection * vx),
                    py - (ay + projection * vy));
}

inline double forkArmDistanceWithSegments(const ForkedForestConfig& config,
                                          const int fork_index,
                                          const ForkArm arm,
                                          const double x_m,
                                          const double y_m,
                                          const int segment_count) {
  const int segments = std::max(1, segment_count);
  const double x0 = config.fork_x_min_m +
                    static_cast<double>(fork_index) * config.fork_length_m;
  const double dx = config.fork_length_m / static_cast<double>(segments);
  double minimum = std::numeric_limits<double>::infinity();
  double previous_x = x0;
  double previous_y = forkArmCenterY(config, fork_index, arm, previous_x);
  for (int segment = 1; segment <= segments; ++segment) {
    const double current_x = x0 + static_cast<double>(segment) * dx;
    const double current_y =
        forkArmCenterY(config, fork_index, arm, current_x);
    minimum = std::min(
        minimum, pointSegmentDistance(x_m, y_m, previous_x, previous_y,
                                      current_x, current_y));
    previous_x = current_x;
    previous_y = current_y;
  }
  return minimum;
}

inline double forkArmDistance(const ForkedForestConfig& config,
                              const int fork_index,
                              const ForkArm arm,
                              const double x_m,
                              const double y_m) {
  return forkArmDistanceWithSegments(
      config, fork_index, arm, x_m, y_m, 128);
}

inline ForkCorridorMembership classifyForkCorridor(
    const ForkedForestConfig& config,
    const double x_m,
    const double y_m) {
  ForkCorridorMembership membership;
  for (int index = 0; index <= kForkedForestForkCount; ++index) {
    const double junction_x = config.fork_x_min_m +
                              static_cast<double>(index) *
                                  config.fork_length_m;
    membership.inside_buffer = membership.inside_buffer ||
        std::hypot(x_m - junction_x, y_m) <=
            config.junction_clearance_radius_m;
  }
  membership.inside_buffer = membership.inside_buffer ||
      std::hypot(x_m - config.start_x_m, y_m) <=
          config.junction_clearance_radius_m ||
      std::hypot(x_m - config.goal_x_m, y_m) <=
          config.junction_clearance_radius_m;

  for (int fork = 0; fork < kForkedForestForkCount; ++fork) {
    if (forkArmDistance(config, fork, ForkArm::kLowRisk, x_m, y_m) <=
        config.corridor_half_width_m) {
      membership.inside_low = true;
      membership.low_fork_index = fork;
    }
    if (forkArmDistance(config, fork, ForkArm::kHighRisk, x_m, y_m) <=
        config.corridor_half_width_m) {
      membership.inside_high = true;
      membership.high_fork_index = fork;
    }
  }
  return membership;
}

inline ForkCorridorMembership classifyForkCorridorEnvelope(
    const ForkedForestConfig& config,
    const double x_m,
    const double y_m) {
  ForkCorridorMembership membership;
  for (int index = 0; index <= kForkedForestForkCount; ++index) {
    const double junction_x = config.fork_x_min_m +
                              static_cast<double>(index) *
                                  config.fork_length_m;
    membership.inside_buffer = membership.inside_buffer ||
        std::hypot(x_m - junction_x, y_m) <=
            config.junction_clearance_radius_m;
  }
  membership.inside_buffer = membership.inside_buffer ||
      std::hypot(x_m - config.start_x_m, y_m) <=
          config.junction_clearance_radius_m ||
      std::hypot(x_m - config.goal_x_m, y_m) <=
          config.junction_clearance_radius_m;

  // A 32-segment local polyline has sub-centimetre sag error for this preset.
  // The small guard prevents point-grid quantization from narrowing the
  // configured tube without clearing the real trees rooted immediately
  // outside it.
  constexpr double kDistanceGuardM = 0.02;
  for (int fork = 0; fork < kForkedForestForkCount; ++fork) {
    const double x0 = config.fork_x_min_m +
                      static_cast<double>(fork) * config.fork_length_m;
    const double x1 = x0 + config.fork_length_m;
    if (x_m < x0 - config.corridor_half_width_m - kDistanceGuardM ||
        x_m > x1 + config.corridor_half_width_m + kDistanceGuardM) {
      continue;
    }
    for (const ForkArm arm : {ForkArm::kLowRisk, ForkArm::kHighRisk}) {
      if (forkArmDistanceWithSegments(
              config, fork, arm, x_m, y_m, 32) >
          config.corridor_half_width_m + kDistanceGuardM) {
        continue;
      }
      if (arm == ForkArm::kLowRisk) {
        membership.inside_low = true;
        membership.low_fork_index = fork;
      } else {
        membership.inside_high = true;
        membership.high_fork_index = fork;
      }
    }
  }
  return membership;
}

}  // namespace iap::sim
