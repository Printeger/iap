#include <iap/util/relinearization_policy.hpp>

#include <iomanip>

namespace glim {

void RelinearizationPolicyRegistry::register_policy(char symbol, int dimension, const gtsam::Vector& threshold) {
  RelinearizationPolicy policy;
  policy.symbol = symbol;
  policy.dimension = dimension;
  policy.threshold = threshold;
  policies_[symbol] = policy;
}

void RelinearizationPolicyRegistry::validate_or_throw() const {
  for (const auto& [symbol, policy] : policies_) {
    if (policy.dimension <= 0) {
      throw std::runtime_error("relinearization policy for symbol '" + std::string(1, symbol) + "' has invalid dimension");
    }

    if (policy.threshold.rows() != policy.dimension) {
      std::ostringstream oss;
      oss << "relinearization policy dimension mismatch for symbol '" << symbol << "': expected "
          << policy.dimension << ", got " << policy.threshold.rows();
      throw std::runtime_error(oss.str());
    }
  }
}

gtsam::FastMap<char, gtsam::Vector> RelinearizationPolicyRegistry::build_map() const {
  gtsam::FastMap<char, gtsam::Vector> relin_map;
  for (const auto& [symbol, policy] : policies_) {
    relin_map[symbol] = policy.threshold;
  }
  return relin_map;
}

std::string RelinearizationPolicyRegistry::summary() const {
  std::ostringstream oss;
  oss << "{";

  bool first = true;
  for (const auto& [symbol, policy] : policies_) {
    if (!first) {
      oss << ", ";
    }
    first = false;

    oss << symbol << ":dim=" << policy.dimension << ":th=[";
    for (int i = 0; i < policy.threshold.rows(); ++i) {
      if (i != 0) {
        oss << ",";
      }
      oss << std::fixed << std::setprecision(3) << policy.threshold(i);
    }
    oss << "]";
  }

  oss << "}";
  return oss.str();
}

}  // namespace glim
