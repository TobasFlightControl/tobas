#pragma once

#include <dh_std_tools/range.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include "./stability_derivatives_cog.hpp"

/**
 * @brief 縦系のトリム状態を求める．
 */
class TrimConditions
{
public:
  explicit TrimConditions(const Drone& drone, uint32_t elev_cs_idx);

  /**
   * @brief 内部状態を更新する．
   *
   * @param V 風速に対する機体速度の絶対値 [m/s]
   * @param h 幾何高度 [m]
   * @param q 関節角 [rad]
   */
  void update(double V, double h, const KDL::JntArray& q);

  const StabilityDerivativesCG& stabilityDerivativesCG() const;

  /* ピッチ回転のトリムに用いる舵面の添字 */
  const uint32_t& elevatorIndex() const;

  /* 迎角 [rad] */
  const double& alpha() const;
  /* ピッチ角 [rad] */
  const double& theta() const;
  /* エレベーター舵角 [rad] */
  const double& elevator() const;
  /* 揚力係数 [-] */
  const double& c_L() const;
  /* 抗力係数 [-] */
  const double& c_D() const;
  /* 推力係数 [-] */
  const double& c_T() const;
  /* X軸方向の速さ [m/s] */
  const double& u() const;

  /* 失速しないための速度の大きさの範囲． */
  dh_std::Range<double> speedLimit(double altitude) const;

private:
  const Drone& drone_;
  const uint32_t elev_cs_idx_;

  KDL::TreeJntToInertiaSolver inertia_solver_;
  StabilityDerivativesCG asd_cog_;

  // 固定値
  double W_;         // 機体の重量 [N]
  double a_, b_;     // (2.9-49)の定数部分

  double alpha_;     // トリム時の迎角 [rad]
  double elevator_;  // トリム時の昇降舵の偏角 [rad]
  double c_L_;       // トリム時の揚力係数 [-]
  double c_D_;       // トリム時の抗力係数 [-]
  double c_T_;       // トリム時の推力係数 [-]
  double u_;         // トリム時のX軸方向の速さ [m/s]
};
