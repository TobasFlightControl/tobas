#pragma once

#include "./drone.hpp"

namespace tobas
{
/**
 * @brief 特定の回転軸を持つロータを抽出する．
 */
class RotorAxisExtractor
{
public:
  explicit RotorAxisExtractor(const Drone& drone, Axis axis);

  void updateInternalDataStructures();

  /* 抽出したロータの個数． */
  uint32_t count() const;

  /* 抽出したロータ配列の添字から元のロータ配列の添字を取得． */
  const uint32_t& rotorIdx(const uint32_t& inner_idx) const;

  /* プロペラのリンク名． */
  const std::string& linkName(const uint32_t& inner_idx) const;

  /* 回転軸． */
  const Axis& axis(const uint32_t& inner_idx) const;

  /* 回転方向: CCW(1) or CW(-1)． */
  const int& direction(const uint32_t& inner_idx) const;

  /* 推力係数 [kg*m/rad^2]． */
  const double& motorConstant(const uint32_t& inner_idx) const;

  /* 反トルク係数 [m]． */
  const double& momentConstant(const uint32_t& inner_idx) const;

  /* 空気効力定数 [kg/rad]． */
  const double& dragConstant(const uint32_t& inner_idx) const;

  /* 回転数と電圧の関係式の係数: V = c1 w + c2 w^2 (V[V], w[rad/s]) */
  const std::pair<double, double>& rotSpeedCoefs(const uint32_t& inner_idx) const;

  /* 指定したロータの推力 [N]． */
  double thrustFromVoltage(const uint32_t& inner_idx, const double& voltage) const;

  /* 指定したロータの回転数から印加電圧を求める． */
  double voltageFromRotSpeed(const uint32_t& inner_idx, const double& rot_speed) const;

  /* 指定したロータの印加電圧から回転数を求める． */
  double rotSpeedFromVoltage(const uint32_t& inner_idx, const double& voltage) const;

  /* 推力 [N] からロータの回転数 [rad/s] を求める． */
  double rotSpeedFromThrust(const uint32_t& inner_idx, const double& thrust) const;

  /* 回転数から合計推力を求める． */
  double thrustSum(const std::vector<double>& rot_speeds);

  /* 最大推力の合計 [N]． */
  double maxThrustSum(const double& battery_voltage) const;

  /* 最小推力の合計 [N]． */
  double minThrustSum(const double& battery_voltage) const;

private:
  const Drone& drone_;
  const Axis axis_;

  std::vector<uint32_t> rotor_idxs_;

  void setRotorIdxs();
};
}  // namespace tobas
