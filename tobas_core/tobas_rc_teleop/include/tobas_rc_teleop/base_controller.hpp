#pragma once

#include <tobas_constants/constants.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

namespace tobas_rc_teleop
{
class BaseController
{
public:
  explicit BaseController();

  virtual bool requirePosition() = 0;
  virtual bool requireVelocity() = 0;
  virtual bool requireAttitude() = 0;
  virtual bool requireHeading() = 0;

  virtual void initialize(tobas::BaseNode* node, tobas::FlightMode mode) = 0;
  virtual void reset(const tobas_msgs::Odometry& odom) = 0;
  virtual void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom) = 0;

protected:
  static constexpr int kExpoScale = 100;
  static constexpr double kDeadband = 0.02;  // S.BUSのジッタは ±2us 程度だから，全帯域の 1% もあれば十分．

  /* デッドバンドに入っていたら 0 にする． */
  inline double deadband(double x) const;

  /* RCInput の値を範囲  [lb, ub] に投影する． */
  inline double remap(double x, double lb, double ub) const;

  /* RCInput の値がデッドバンドに入っていたら0，入っていなければ [lb, ub] に投影する． */
  inline double remapDead(double x, double lb, double ub) const;

  /* expo -> remap */
  inline double expoRemap(double x, double exp, double lb, double ub) const;

  /* dead -> expo -> remap */
  inline double expoRemapDead(double x, double exp, double lb, double ub) const;

  /* FutabaのEXPO関数とたぶん同じ: [-1, 1] -> [-1, 1] */
  static inline double expo(double x, double exp);

  /* テキストにフライトモードのプリフィックスを与える． */
  static std::string addMode(const std::string& text, tobas::FlightMode mode);
};

inline double BaseController::deadband(double x) const
{
  return std::abs(x) < kDeadband ? 0. : x;
}

inline double BaseController::remap(double x, double lb, double ub) const
{
  return math::remap(x, tobas::kRcInputMin, tobas::kRcInputMax, lb, ub);
}

inline double BaseController::expoRemap(double x, double exp, double lb, double ub) const
{
  return remap(expo(x, exp), lb, ub);
}

inline double BaseController::remapDead(double x, double lb, double ub) const
{
  return remap(deadband(x), lb, ub);
}

inline double BaseController::expoRemapDead(double x, double exp, double lb, double ub) const
{
  return expoRemap(deadband(x), exp, lb, ub);
}

inline double BaseController::expo(double x, double exp)
{
  assert(std::abs(x) < 1.);
  assert(std::abs(exp) < 1.);
  return (1. + exp) * x - exp * math::sign(x) * math::sqr(x);
}
}  // namespace tobas_rc_teleop
