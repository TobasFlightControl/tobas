#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
class EngineConfig
{
  static constexpr char kTorqueConstantKey[] = "torque_constant";
  static constexpr char kDynamicFrictionTorqueKey[] = "dynamic_friction_torque";

public:
  double torque_const = 0.;     // [Nm/(rad/s)]
  double friction_torque = 0.;  // [Nm]

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  /* Compute engine throttle [0, 1] from torque [N] and speed [rad/s]. */
  double solveThrottleEquation(double torque, double speed);
};
}  // namespace tobas
