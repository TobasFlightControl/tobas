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

  /* 効率を考慮した1ボルトあたりの回転数 [rpm/V]． */
  const double& kv(uint32_t inner_idx) const;

  /* 最大回転数 [rad/s]． */
  double maxRotSpeed(uint32_t inner_idx, double battery_voltage) const;

  /* 最大推力 [N]． */
  double maxThrust(uint32_t inner_idx, double battery_voltage) const;

  /* 最小推力 [N]． */
  double minThrust(uint32_t inner_idx, double battery_voltage) const;

  /* 最大推力の合計 [N]． */
  double maxThrustSum(double battery_voltage) const;

  /* 最小推力の合計 [N]． */
  double minThrustSum(double battery_voltage) const;

  /* 推力 [N] からロータの回転数 [rad/s] を求める． */
  double thrustToRotSpeed(uint32_t inner_idx, double thrust) const;

private:
  const Drone& drone_;
  const Axis axis_;

  std::vector<uint32_t> rotor_idxs_;

  void setRotorIdxs();
};
}  // namespace tobas
