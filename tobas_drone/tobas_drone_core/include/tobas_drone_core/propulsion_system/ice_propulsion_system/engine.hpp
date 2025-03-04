#pragma once

#include <iostream>
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

  /* Compute engine torque [Nm] from speed [rad/s] and throttle [0, 1]. */
  double computeTorque(double speed, double throttle);

  /* Compute engine speed [rad/s] from throttle [0, 1] and torque [Nm]. */
  double computeSpeed(double throttle, double torque);

  /* Compute engine throttle [0, 1] from torque [N] and speed [rad/s]. */
  double computeThrottle(double torque, double speed);

  friend std::ostream& operator<<(std::ostream& os, const EngineConfig& arg);

private:
  static double g(double throttle);
};
}  // namespace tobas
