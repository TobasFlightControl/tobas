#pragma once

#include <tobas_drone_core/drone.hpp>

#include "./solveri.hpp"

namespace tobas
{
/**
 * @brief 特定の回転軸を持つロータを抽出する．
 */
class RotorAxisExtractor : public SolverI
{
public:
  explicit RotorAxisExtractor(const Drone& drone, const rotor_axis_t& axis);

  void updateInternalDataStructures() override;

  /* 抽出したロータの個数． */
  inline const size_t& count() const;

  /* 抽出したロータ配列の添字から元のロータ配列の添字を取得． */
  inline const size_t& rotorIdx(const size_t& inner_idx) const;

  /* モータが接続されているチャンネル． */
  inline const uint32_t& channel(const size_t& inner_idx) const;

  /* プロペラのリンク名． */
  inline const std::string& linkName(const size_t& inner_idx) const;

  /* 回転軸． */
  inline const rotor_axis_t& axis(const size_t& inner_idx) const;

  /* 回転方向: CCW(1) or CW(-1)． */
  inline int sign(const size_t& inner_idx) const;

  /* 推力係数 [kg*m/rad^2]． */
  inline const double& motorConstant(const size_t& inner_idx) const;

  /* 反トルク係数 [m]． */
  inline const double& momentConstant(const size_t& inner_idx) const;

  /* 空気効力定数 [kg/rad]． */
  inline const double& dragConstant(const size_t& inner_idx) const;

  /* 機械的に許容できる最大回転数から計算される推力． */
  inline double maxMechanicalThrust(const size_t& inner_idx) const;

  /* 与えられたバッテリー電圧で出力できる最大推力．*/
  inline double maxThrust(const size_t& inner_idx, const double& battery_voltage) const;

  /* 与えられたバッテリー電圧で出力できる最小推力．*/
  inline double minThrust(const size_t& inner_idx, const double& battery_voltage) const;

  /* 指定したロータの推力 [N]． */
  inline double thrustFromVoltage(const size_t& inner_idx, const double& voltage) const;

  /* 指定したロータの回転数から印加電圧を求める． */
  inline double voltageFromRotSpeed(const size_t& inner_idx, const double& rot_speed) const;

  /* 指定したロータの印加電圧から回転数を求める． */
  inline double rotSpeedFromVoltage(const size_t& inner_idx, const double& voltage) const;

  /* 推力 [N] からロータの回転数 [rad/s] を求める． */
  inline double rotSpeedFromThrust(const size_t& inner_idx, const double& thrust) const;

  /* 推力 [N] からスロットル [0,1] を求める． */
  inline double throttleFromThrust(const size_t& inner_idx, const double& thrust, const double& battery_voltage);

  /* 各ロータの回転数から合計推力を求める． */
  double thrustSum(const std::vector<double>& rot_speeds) const;

  /* スロットル [0,1] から合計推力を求める． */
  double thrustSum(const double& battery_voltage, const double& throttle);

  /* 最大推力の合計 [N]． */
  double maxThrustSum(const double& battery_voltage) const;

  /* 最小推力の合計 [N]． */
  double minThrustSum(const double& battery_voltage) const;

private:
  const Drone& drone_;
  const rotor_axis_t axis_;

  size_t count_;  // The number of rotors with the specified rotation axis
  std::vector<size_t> rotor_idxs_;
};

inline const size_t& RotorAxisExtractor::count() const
{
  return count_;
}

inline const size_t& RotorAxisExtractor::rotorIdx(const size_t& inner_idx) const
{
  return rotor_idxs_.at(inner_idx);
}

inline const uint32_t& RotorAxisExtractor::channel(const size_t& inner_idx) const
{
  return drone_.rotors.at(rotorIdx(inner_idx)).channel;
}

inline const std::string& RotorAxisExtractor::linkName(const size_t& inner_idx) const
{
  return drone_.rotors.at(rotorIdx(inner_idx)).link_name;
}

inline const rotor_axis_t& RotorAxisExtractor::axis(const size_t& inner_idx) const
{
  return drone_.rotors.at(rotorIdx(inner_idx)).axis;
}

inline int RotorAxisExtractor::sign(const size_t& inner_idx) const
{
  return drone_.rotors.at(rotorIdx(inner_idx)).sign();
}

inline const double& RotorAxisExtractor::motorConstant(const size_t& inner_idx) const
{
  return drone_.rotors.at(rotorIdx(inner_idx)).motor_constant;
}

inline const double& RotorAxisExtractor::momentConstant(const size_t& inner_idx) const
{
  return drone_.rotors.at(rotorIdx(inner_idx)).moment_constant;
}

inline const double& RotorAxisExtractor::dragConstant(const size_t& inner_idx) const
{
  return drone_.rotors.at(rotorIdx(inner_idx)).drag_constant;
}

inline double RotorAxisExtractor::maxMechanicalThrust(const size_t& inner_idx) const
{
  return drone_.maxMechanicalThrust(rotorIdx(inner_idx));
}

inline double RotorAxisExtractor::maxThrust(const size_t& inner_idx, const double& battery_voltage) const
{
  return drone_.maxThrust(rotorIdx(inner_idx), battery_voltage);
}

inline double RotorAxisExtractor::minThrust(const size_t& inner_idx, const double& battery_voltage) const
{
  return drone_.minThrust(rotorIdx(inner_idx), battery_voltage);
}

inline double RotorAxisExtractor::thrustFromVoltage(const size_t& inner_idx, const double& voltage) const
{
  return drone_.thrustFromVoltage(rotorIdx(inner_idx), voltage);
}

inline double RotorAxisExtractor::voltageFromRotSpeed(const size_t& inner_idx, const double& rot_speed) const
{
  return drone_.voltageFromRotSpeed(rotorIdx(inner_idx), rot_speed);
}

inline double RotorAxisExtractor::rotSpeedFromVoltage(const size_t& inner_idx, const double& voltage) const
{
  return drone_.rotSpeedFromVoltage(rotorIdx(inner_idx), voltage);
}

inline double RotorAxisExtractor::rotSpeedFromThrust(const size_t& inner_idx, const double& thrust) const
{
  return drone_.rotSpeedFromThrust(rotorIdx(inner_idx), thrust);
}

inline double
RotorAxisExtractor::throttleFromThrust(const size_t& inner_idx, const double& thrust, const double& battery_voltage)
{
  return drone_.throttleFromThrust(rotorIdx(inner_idx), thrust, battery_voltage);
}
}  // namespace tobas
