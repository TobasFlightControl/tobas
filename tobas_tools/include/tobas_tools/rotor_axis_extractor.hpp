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
  inline const uint32_t& count() const;

  /* 抽出したロータ配列の添字から元のロータ配列の添字を取得． */
  inline const uint32_t& rotorIdx(const uint32_t& inner_idx) const;

  /* プロペラのリンク名． */
  inline const std::string& linkName(const uint32_t& inner_idx) const;

  /* 回転軸． */
  inline const Axis& axis(const uint32_t& inner_idx) const;

  /* 回転方向: CCW(1) or CW(-1)． */
  inline const int& direction(const uint32_t& inner_idx) const;

  /* 推力係数 [kg*m/rad^2]． */
  inline const double& motorConstant(const uint32_t& inner_idx) const;

  /* 反トルク係数 [m]． */
  inline const double& momentConstant(const uint32_t& inner_idx) const;

  /* 空気効力定数 [kg/rad]． */
  inline const double& dragConstant(const uint32_t& inner_idx) const;

  /* 回転数と電圧の関係式の係数: V = c1 w + c2 w^2 (V[V], w[rad/s]) */
  inline const std::pair<double, double>& rotSpeedCoefs(const uint32_t& inner_idx) const;

  /* 指定したロータの推力 [N]． */
  inline double thrustFromVoltage(const uint32_t& inner_idx, const double& voltage) const;

  /* 指定したロータの回転数から印加電圧を求める． */
  inline double voltageFromRotSpeed(const uint32_t& inner_idx, const double& rot_speed) const;

  /* 指定したロータの印加電圧から回転数を求める． */
  inline double rotSpeedFromVoltage(const uint32_t& inner_idx, const double& voltage) const;

  /* 推力 [N] からロータの回転数 [rad/s] を求める． */
  inline double rotSpeedFromThrust(const uint32_t& inner_idx, const double& thrust) const;

  /* 回転数から合計推力を求める． */
  double thrustSum(const std::vector<double>& rot_speeds) const;

  /* 最大推力の合計 [N]． */
  double maxThrustSum(const double& battery_voltage) const;

  /* 最小推力の合計 [N]． */
  double minThrustSum(const double& battery_voltage) const;

private:
  const Drone& drone_;
  const Axis axis_;

  uint32_t count_;  // The number of rotors with the specified rotation axis
  std::vector<uint32_t> rotor_idxs_;
};

inline const uint32_t& RotorAxisExtractor::count() const
{
  return count_;
}

inline const uint32_t& RotorAxisExtractor::rotorIdx(const uint32_t& inner_idx) const
{
  return rotor_idxs_[inner_idx];
}

inline const std::string& RotorAxisExtractor::linkName(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).link_name;
}

inline const Axis& RotorAxisExtractor::axis(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).axis;
}

inline const int& RotorAxisExtractor::direction(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).direction;
}

inline const double& RotorAxisExtractor::motorConstant(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).motor_constant;
}

inline const double& RotorAxisExtractor::momentConstant(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).moment_constant;
}

inline const double& RotorAxisExtractor::dragConstant(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).drag_constant;
}

inline const std::pair<double, double>&
RotorAxisExtractor::rotSpeedCoefs(const uint32_t& inner_idx) const
{
  return drone_.rotorConfig(rotorIdx(inner_idx)).rot_speed_coefs;
}

inline double
RotorAxisExtractor::thrustFromVoltage(const uint32_t& inner_idx, const double& voltage) const
{
  return drone_.thrustFromVoltage(rotorIdx(inner_idx), voltage);
}

inline double
RotorAxisExtractor::voltageFromRotSpeed(const uint32_t& inner_idx, const double& rot_speed) const
{
  return drone_.voltageFromRotSpeed(rotorIdx(inner_idx), rot_speed);
}

inline double
RotorAxisExtractor::rotSpeedFromVoltage(const uint32_t& inner_idx, const double& voltage) const
{
  return drone_.rotSpeedFromVoltage(rotorIdx(inner_idx), voltage);
}

inline double
RotorAxisExtractor::rotSpeedFromThrust(const uint32_t& inner_idx, const double& thrust) const
{
  return drone_.rotSpeedFromThrust(rotorIdx(inner_idx), thrust);
}
}  // namespace tobas
