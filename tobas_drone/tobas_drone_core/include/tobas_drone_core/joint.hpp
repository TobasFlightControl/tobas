#pragma once

#include <string>
#include <map>

namespace tobas
{
struct JointConfig
{
  double home_pos;  // [rad | m]
  double min_pos;   // [rad | m]
  double max_pos;   // [rad | m]

  enum command_type_t : u_int8_t
  {
    POSITION,
    VELOCITY,
    EFFORT,
  } cmd_type;
};

using JointConfigMap = std::map<std::string, JointConfig>;
}  // namespace tobas
