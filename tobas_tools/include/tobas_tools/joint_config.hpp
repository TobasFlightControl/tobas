#pragma once

#include <string>
#include <vector>

namespace tobas
{
struct JointConfig
{
  std::string name;
  double init_pos;
  double min_pos;
  double max_pos;
};

using JointConfigs = std::vector<JointConfig>;
}  // namespace tobas
