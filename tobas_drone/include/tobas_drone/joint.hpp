#pragma once

#include <string>
#include <map>

namespace tobas
{
struct JointConfig
{
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

using JointConfigMap = std::map<std::string, JointConfig>;
}  // namespace tobas
