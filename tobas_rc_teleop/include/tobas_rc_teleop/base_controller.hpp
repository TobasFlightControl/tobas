#pragma once

#include <tobas_math/core.hpp>
#include <tobas_std_tools/range.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

namespace tobas_rc_teleop
{
class BaseController
{
public:
  explicit BaseController();

  virtual bool requirePosition() = 0;
  virtual bool requireOrientation() = 0;
  virtual bool requireLinearVelocity() = 0;
  virtual bool requireAngularVelocity() = 0;

  virtual void initialize(tobas::BaseNode* node, tobas::flight_mode_t mode) = 0;
  virtual void reset(const tobas_msgs::Odometry& odom) = 0;
  virtual void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) = 0;

protected:
  static constexpr int kExpoScale = 100;
  static constexpr double kDeadZone = 0.05;

  const tobas_std::Range<double> dead_zone_;

  /* dead_zoneに入っているかどうか． */
  inline bool inDeadZone(const double& x) const;

  /* RCInputの値を範囲[lb, ub]に投影する． */
  inline double remap(const double& x, const double& lb, const double& ub) const;

  /* RCInputの値がdead_zoneに入っていたら0，入っていなければ[lb, ub]に投影する． */
  inline double remapDead(const double& x, const double& lb, const double& ub) const;

  /* expo -> remap */
  inline double expoRemap(const double& x, const double& exp, const double& lb, const double& ub) const;

  /* dead -> expo -> remap */
  inline double expoRemapDead(const double& x, const double& exp, const double& lb, const double& ub) const;

  /* FutabaのEXPO関数とたぶん同じ: [-1, 1] -> [-1, 1] */
  static inline double expo(const double& x, const double& exp);

  /* テキストにフライトモードのプリフィックスを与える． */
  static std::string addMode(const std::string& text, tobas::flight_mode_t mode);
};

inline bool BaseController::inDeadZone(const double& x) const
{
  return dead_zone_.inRange(x);
}

inline double BaseController::remap(const double& x, const double& lb, const double& ub) const
{
  return math::remap(x, tobas::kRcInputMin, tobas::kRcInputMax, lb, ub);
}

inline double BaseController::expoRemap(const double& x, const double& exp, const double& lb, const double& ub) const
{
  return remap(expo(x, exp), lb, ub);
}

inline double BaseController::remapDead(const double& x, const double& lb, const double& ub) const
{
  return inDeadZone(x) ? 0. : remap(x, lb, ub);
}

inline double BaseController::expoRemapDead(const double& x, const double& exp, const double& lb, const double& ub) const
{
  return inDeadZone(x) ? 0. : expoRemap(x, exp, lb, ub);
}

inline double BaseController::expo(const double& x, const double& exp)
{
  assert(abs(x) < 1.);
  assert(abs(exp) < 1.);
  return (1. + exp) * x - exp * math::sign(x) * math::sqr(x);
}
}  // namespace tobas_rc_teleop
