#pragma once

#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtsam/base/FastMap.h>
#include <gtsam/base/Vector.h>

namespace glim {

struct RelinearizationPolicy {
  char symbol = '\0';
  int dimension = 0;
  gtsam::Vector threshold;
};

class RelinearizationPolicyRegistry {
public:
  void register_policy(char symbol, int dimension, const gtsam::Vector& threshold);

  void validate_or_throw() const;

  gtsam::FastMap<char, gtsam::Vector> build_map() const;

  std::string summary() const;

private:
  std::map<char, RelinearizationPolicy> policies_;
};

}  // namespace glim
