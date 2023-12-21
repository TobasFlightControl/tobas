#pragma once

#include <string>
#include <vector>

namespace tobas
{
struct JointConfig
{
  std::string name;
  double home_pos;
  double min_pos;
  double max_pos;

  enum command_type_t
  {
    POSITION,
    VELOCITY,
    EFFORT,
  } cmd_type;
};

using JointConfigs = std::vector<JointConfig>;
}  // namespace tobas
