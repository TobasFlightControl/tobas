#pragma once

#include <tobas_drone_core/drone.hpp>

#include "./solver_i.hpp"

namespace tobas
{
/**
 * @brief 特定の回転軸を持つロータを抽出する．
 */
class RotorAxisExtractor : public SolverI
{
public:
  explicit RotorAxisExtractor(const Drone& drone, const rotor_axis_t& axis);

  bool updateInternalDataStructures() override;

  inline size_t count() const;
  inline const RotorConfig& rotor(size_t idx) const;

  /* 各ロータの回転数から合計推力を求める． */
  double thrustSum(const std::vector<double>& rot_speeds) const;

  /* スロットル [0,1] から合計推力を求める． */
  double thrustSum(const double& battery_voltage, const double& throttle);

private:
  const Drone& drone_;
  const rotor_axis_t axis_;

  std::vector<size_t> channels_;

  void initialize();
};

inline size_t RotorAxisExtractor::count() const
{
  return channels_.size();
}

inline const RotorConfig& RotorAxisExtractor::rotor(size_t idx) const
{
  const auto& channel = channels_.at(idx);
  return drone_.rotors.at(channel);
}
}  // namespace tobas
