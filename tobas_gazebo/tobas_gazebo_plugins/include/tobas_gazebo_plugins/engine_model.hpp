#pragma once

#include "./filter/asymmetric_first_order_filter.hpp"
#include "./ice_rotor_model.hpp"

namespace gazebo
{
class EngineModel
{
public:
  explicit EngineModel(const ICERotorModelMap& rotors);

  bool initialize(const sdf::ElementConstPtr& sdf);

  double getTorqueConst() const;
  double getFrictionTorque() const;

  /* 回転数 [rad/s] */
  double getSpeed() const;

  /* 回転位置 [rad] */
  double getPosition() const;

  void setThrottle(const double& throttle);

  bool step(const double& dt);

private:
  const ICERotorModelMap& rotors_;

  // SDF parameters
  double torque_const_;     // [Nm/(rad/s)]
  double friction_torque_;  // [Nm]
  double time_const_up_;    // [s]
  double time_const_down_;  // [s]

  // Command
  double throttle_ = 0.;  // スロットル開度 [0, 1]

  // State
  double position_ = 0.;  // 位置 [rad]
  AsymmetricFirstOrderFilter<double> speed_filter_;

  bool getSdfParams(const sdf::ElementConstPtr& sdf);

  /* エンジンスロットルとティルト角から定常回転数を求める (memo: 3-26) */
  double computeSteadySpeed();
};
}  // namespace gazebo
