#pragma once

#include <tobas_std_tools/range.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_drone_core/drone.hpp>

#include "./solver_i.hpp"
#include "./fw_stability_derivatives_cog.hpp"

namespace tobas
{
/**
 * @brief 縦系のトリム状態を求める．
 */
class TrimConditions : public SolverI
{
public:
  explicit TrimConditions(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  /**
   * @brief 内部状態を更新する．
   *
   * @param V 風速に対する機体速度の絶対値 [m/s]
   * @param rho 大気密度 [kg/m^3]
   * @param q 関節角 [rad]
   *
   * @return ErrorCode Error code
   */
  int update(double V, const double& rho, const kdl::JntArray& q);

  inline const StabilityDerivativesCG& stabilityDerivativesCG() const;

  /* ピッチ回転のトリムに用いる舵面の添字 */
  inline const size_t& elevatorChannel() const;

  /* 迎角 [rad] */
  inline const double& alpha() const;
  /* ピッチ角 [rad] */
  inline const double& theta() const;
  /* エレベーター舵角 [rad] */
  inline const double& elevator() const;
  /* 揚力係数 [-] */
  inline const double& c_L() const;
  /* 抗力係数 [-] */
  inline const double& c_D() const;
  /* 推力係数 [-] */
  inline const double& c_T() const;
  /* X軸方向の速さ [m/s] */
  inline const double& u() const;

  inline double minimumSpeed(const double& rho) const;
  inline double maximumSpeed(const double& rho) const;

  /**
   * @brief 失速しないための速度の大きさの範囲．
   * cf. 青本, p.85, (2.9-47, 2.9-49)
   *
   * @param rho 大気密度 [kg/m^3]
   * @return tobas_std::Range<double> 速度の大きさの範囲
   */
  tobas_std::Range<double> speedLimit(const double& rho) const;

  /* 迎角が0でも機体を持ち上げるだけの揚力が発生する速度． */
  double takeOffSpeed(const double& rho) const;

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeInertiaSolver inertia_solver_;
  StabilityDerivativesCG asd_cog_;

  // 固定値
  double W_;             // 機体の重量 [N]
  size_t elev_channel_;  // ピッチ回転の釣り合いに使う舵面のチャンネル
  double a_, b_;         // (2.9-49)の定数部分

  double alpha_;     // トリム時の迎角 [rad]
  double elevator_;  // トリム時の昇降舵の偏角 [rad]
  double c_L_;       // トリム時の揚力係数 [-]
  double c_D_;       // トリム時の抗力係数 [-]
  double c_T_;       // トリム時の推力係数 [-]
  double u_;         // トリム時のX軸方向の速さ [m/s]
};

inline const StabilityDerivativesCG& TrimConditions::stabilityDerivativesCG() const
{
  return asd_cog_;
}

inline const size_t& TrimConditions::elevatorChannel() const
{
  return elev_channel_;
}

inline const double& TrimConditions::alpha() const
{
  return alpha_;
}

inline const double& TrimConditions::theta() const
{
  return alpha_;  // 水平飛行より theta = alpha
}

inline const double& TrimConditions::elevator() const
{
  return elevator_;
}

inline const double& TrimConditions::c_L() const
{
  return c_L_;
}

inline const double& TrimConditions::c_D() const
{
  return c_D_;
}

inline const double& TrimConditions::c_T() const
{
  return c_T_;
}

inline const double& TrimConditions::u() const
{
  return u_;
}

inline double TrimConditions::minimumSpeed(const double& rho) const
{
  return speedLimit(rho).lower;
}

inline double TrimConditions::maximumSpeed(const double& rho) const
{
  return speedLimit(rho).upper;
}
}  // namespace tobas
