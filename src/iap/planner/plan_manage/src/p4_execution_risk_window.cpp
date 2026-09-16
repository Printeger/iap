#include "ego_planner/p4_execution_risk_window.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <utility>

namespace ego_planner {
namespace {

constexpr double kTimeToleranceS = 1.0e-9;

std::string sampleKey(const P4ExecutionRiskSample& sample) {
  std::ostringstream key;
  key << std::hexfloat << sample.relative_time_s << ';'
      << sample.position.x() << ';' << sample.position.y() << ';'
      << sample.position.z();
  return key.str();
}

std::string fnvHash(const std::string& canonical) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : canonical) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

bool validSample(const P4ExecutionRiskSample& sample) {
  return sample.position.allFinite() &&
      std::isfinite(sample.relative_time_s) &&
      sample.relative_time_s >= -kTimeToleranceS;
}

}  // namespace

P4ExecutionRiskWindowLayout buildP4ExecutionRiskWindowLayout(
    const std::vector<P4ExecutionRiskSample>& nominal_samples,
    const std::vector<P4BrakingRiskCurveSamples>& braking_curves,
    const P4ExecutionRiskWindowParams& params) {
  P4ExecutionRiskWindowLayout out;
  if (nominal_samples.size() < 2u || braking_curves.empty() ||
      !std::isfinite(params.reaction_time_s) ||
      params.reaction_time_s <= 0.0 ||
      !std::isfinite(params.transition_overlap_s) ||
      params.transition_overlap_s < 0.0 ||
      !std::isfinite(params.maximum_anchor_gap_s) ||
      params.maximum_anchor_gap_s <= 0.0) {
    out.reason = "invalid_window_input";
    return out;
  }
  for (std::size_t index = 0; index < nominal_samples.size(); ++index) {
    if (!validSample(nominal_samples[index]) ||
        (index > 0 && nominal_samples[index].relative_time_s +
                 kTimeToleranceS <
             nominal_samples[index - 1].relative_time_s)) {
      out.reason = "invalid_nominal_sample_lattice";
      return out;
    }
  }
  const double start_time_s = nominal_samples.front().relative_time_s;
  const double end_time_s = nominal_samples.back().relative_time_s;
  if (end_time_s <= start_time_s + kTimeToleranceS) {
    out.reason = "invalid_nominal_duration";
    return out;
  }

  std::vector<std::size_t> brake_order(braking_curves.size());
  for (std::size_t index = 0; index < brake_order.size(); ++index) {
    brake_order[index] = index;
  }
  std::sort(brake_order.begin(), brake_order.end(),
            [&braking_curves](const std::size_t lhs,
                              const std::size_t rhs) {
              return braking_curves[lhs].anchor_time_s <
                  braking_curves[rhs].anchor_time_s;
            });
  for (std::size_t ordered_index = 0;
       ordered_index < brake_order.size(); ++ordered_index) {
    const auto& curve = braking_curves[brake_order[ordered_index]];
    if (!std::isfinite(curve.anchor_time_s) || curve.samples.empty() ||
        curve.anchor_time_s < start_time_s - kTimeToleranceS ||
        curve.anchor_time_s > end_time_s + kTimeToleranceS) {
      out.reason = "invalid_braking_curve";
      return out;
    }
    double previous_time_s = curve.anchor_time_s;
    for (std::size_t sample_index = 0;
         sample_index < curve.samples.size(); ++sample_index) {
      const auto& sample = curve.samples[sample_index];
      if (!validSample(sample) ||
          sample.relative_time_s + kTimeToleranceS < previous_time_s ||
          (sample_index == 0 &&
           std::abs(sample.relative_time_s - curve.anchor_time_s) >
               kTimeToleranceS)) {
        out.reason = "invalid_braking_curve_sample_lattice";
        return out;
      }
      previous_time_s = sample.relative_time_s;
    }
    if (ordered_index > 0) {
      const double previous_anchor =
          braking_curves[brake_order[ordered_index - 1]].anchor_time_s;
      if (curve.anchor_time_s - previous_anchor >
          params.maximum_anchor_gap_s + kTimeToleranceS) {
        out.reason = "braking_anchor_gap_exceeds_limit";
        return out;
      }
    }
  }
  const double first_anchor =
      braking_curves[brake_order.front()].anchor_time_s;
  const double last_anchor =
      braking_curves[brake_order.back()].anchor_time_s;
  if (first_anchor - start_time_s >
          params.maximum_anchor_gap_s + kTimeToleranceS ||
      end_time_s - last_anchor >
          params.maximum_anchor_gap_s + kTimeToleranceS) {
    out.reason = "braking_anchor_endpoint_gap";
    return out;
  }

  std::vector<double> boundaries{start_time_s};
  while (end_time_s - boundaries.back() >
         params.reaction_time_s + params.maximum_anchor_gap_s) {
    const double target = boundaries.back() + params.reaction_time_s;
    double best_anchor = std::numeric_limits<double>::quiet_NaN();
    double best_error = std::numeric_limits<double>::infinity();
    for (const std::size_t curve_index : brake_order) {
      const double anchor = braking_curves[curve_index].anchor_time_s;
      if (anchor <= boundaries.back() + kTimeToleranceS ||
          anchor >= end_time_s - kTimeToleranceS) {
        continue;
      }
      const double error = std::abs(anchor - target);
      if (error < best_error) {
        best_error = error;
        best_anchor = anchor;
      }
    }
    if (!std::isfinite(best_anchor) ||
        best_error > params.maximum_anchor_gap_s + kTimeToleranceS) {
      out.reason = "transition_anchor_snap_exceeds_limit";
      return out;
    }
    boundaries.push_back(best_anchor);
  }
  boundaries.push_back(end_time_s);

  const double overlap_half_s = 0.5 * params.transition_overlap_s;
  out.windows.reserve(boundaries.size() - 1u);
  for (std::size_t index = 0; index + 1u < boundaries.size(); ++index) {
    P4ExecutionRiskWindow window;
    window.window_id = static_cast<std::uint64_t>(index + 1u);
    window.nominal_start_time_s = boundaries[index];
    window.nominal_end_time_s = boundaries[index + 1u];
    window.certified_start_time_s = std::max(
        start_time_s, window.nominal_start_time_s - overlap_half_s);
    window.certified_end_time_s = std::min(
        end_time_s, window.nominal_end_time_s + overlap_half_s);
    out.windows.push_back(std::move(window));
  }
  out.transition_count = out.windows.empty() ? 0u : out.windows.size() - 1u;

  std::map<std::string, std::uint64_t> evidence_ids;
  std::map<std::pair<std::uint64_t, std::uint64_t>, std::size_t> row_lookup;
  std::vector<std::set<std::uint64_t>> evidence_memberships(1u);
  const auto add_sample = [&](const P4ExecutionRiskSample& sample,
                              const std::size_t window_index,
                              const bool nominal,
                              const bool braking,
                              const std::size_t braking_curve_index,
                              P4ExecutionRiskWindowLayout* layout) {
    const std::string key = sampleKey(sample);
    auto inserted = evidence_ids.emplace(
        key, static_cast<std::uint64_t>(evidence_ids.size() + 1u));
    const std::uint64_t evidence_id = inserted.first->second;
    if (evidence_memberships.size() <= evidence_id) {
      evidence_memberships.resize(static_cast<std::size_t>(evidence_id + 1u));
    }
    const std::uint64_t window_id = layout->windows[window_index].window_id;
    evidence_memberships[evidence_id].insert(window_id);
    const auto membership = std::make_pair(evidence_id, window_id);
    auto existing = row_lookup.find(membership);
    if (existing != row_lookup.end()) {
      auto& row = layout->rows[existing->second];
      row.nominal = row.nominal || nominal;
      row.braking = row.braking || braking;
      if (braking && row.braking_curve_index ==
                          std::numeric_limits<std::size_t>::max()) {
        row.braking_curve_index = braking_curve_index;
      }
      return;
    }
    P4ExecutionRiskWindowQueryRow row;
    row.sample = sample;
    row.evidence_point_id = evidence_id;
    row.satellite_window_id = window_id;
    row.nominal = nominal;
    row.braking = braking;
    row.braking_curve_index = braking_curve_index;
    const std::size_t row_index = layout->rows.size();
    layout->rows.push_back(std::move(row));
    row_lookup.emplace(membership, row_index);
    layout->windows[window_index].request_row_indices.push_back(row_index);
  };

  for (std::size_t window_index = 0;
       window_index < out.windows.size(); ++window_index) {
    const auto& window = out.windows[window_index];
    for (const auto& sample : nominal_samples) {
      if (sample.relative_time_s + kTimeToleranceS >=
              window.certified_start_time_s &&
          sample.relative_time_s <=
              window.certified_end_time_s + kTimeToleranceS) {
        add_sample(sample, window_index, true, false,
                   std::numeric_limits<std::size_t>::max(), &out);
      }
    }
    for (const std::size_t curve_index : brake_order) {
      const auto& curve = braking_curves[curve_index];
      if (curve.anchor_time_s + kTimeToleranceS <
              window.certified_start_time_s ||
          curve.anchor_time_s >
              window.certified_end_time_s + kTimeToleranceS) {
        continue;
      }
      for (const auto& sample : curve.samples) {
        add_sample(sample, window_index, false, true, curve_index, &out);
      }
    }
  }

  for (auto& row : out.rows) {
    row.transition_overlap =
        evidence_memberships[row.evidence_point_id].size() > 1u;
  }
  out.unique_evidence_point_count = evidence_ids.size();
  if (out.rows.empty()) {
    out.reason = "empty_window_layout";
    return out;
  }

  std::ostringstream identity;
  identity << std::hexfloat << "p4_braking_window_layout_v1;"
           << params.reaction_time_s << ';' << params.transition_overlap_s
           << ';' << params.maximum_anchor_gap_s << ';';
  for (const auto& window : out.windows) {
    identity << window.window_id << ';' << window.nominal_start_time_s << ';'
             << window.nominal_end_time_s << ';'
             << window.certified_start_time_s << ';'
             << window.certified_end_time_s << ';';
  }
  for (const auto& row : out.rows) {
    identity << row.evidence_point_id << ';' << row.satellite_window_id
             << ';' << sampleKey(row.sample) << ';' << row.nominal << ';'
             << row.braking << ';';
  }
  out.identity_hash = fnvHash(identity.str());
  out.valid = true;
  out.reason = "complete";
  return out;
}

}  // namespace ego_planner
