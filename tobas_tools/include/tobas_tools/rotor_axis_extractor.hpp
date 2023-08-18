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
  const uint32_t& rotorIdx(uint32_t inner_idx) const;

  /* プロペラのリンク名． */
  const std::string& linkName(uint32_t inner_idx) const;

  /* 回転軸． */
  const Axis& axis(uint32_t inner_idx) const;

  /* 回転方向: CCW(1) or CW(-1)． */
  const int& direction(uint32_t inner_idx) const;

  /* 推力係数 [N*s^2/rad^2]． */
  const double& motorConstant(uint32_t inner_idx) const;

  /* 反トルク係数 [m]． */
  const double& momentConstant(uint32_t inner_idx) const;

  /* 回転数と電圧の関係式の係数: V = c1 w + c2 w^2 (V[V], w[rad/s]) */
  const std::pair<double, double>& rotSpeedCoefs(uint32_t inner_idx) const;

  /* 指定したロータの推力 [N]． */
  double thrustFromVoltage(uint32_t inner_idx, double voltage) const;

  /* 指定したロータの回転数から印加電圧を求める． */
  double voltageFromRotSpeed(uint32_t inner_idx, double rot_speed) const;

  /* 指定したロータの印加電圧から回転数を求める． */
  double rotSpeedFromVoltage(uint32_t inner_idx, double voltage) const;

  /* 推力 [N] からロータの回転数 [rad/s] を求める． */
  double rotSpeedFromThrust(uint32_t inner_idx, double thrust) const;

  /* 最大推力の合計 [N]． */
  double maxThrustSum(double battery_voltage) const;

  /* 最小推力の合計 [N]． */
  double minThrustSum(double battery_voltage) const;

private:
  const Drone& drone_;
  const Axis axis_;

  std::vector<uint32_t> rotor_idxs_;

  void setRotorIdxs();
};
}  // namespace tobas
