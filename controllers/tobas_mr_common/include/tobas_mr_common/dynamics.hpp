#pragma once

#include <dh_kdl/treefksolverpos.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>

namespace tobas_mr_common
{
/* 平面配置マルチコプターの運動方程式の要素． */
class MultirotorDynamicsComponents
{
public:
  explicit MultirotorDynamicsComponents(const tobas::Drone& tobas);

  void updateInternalDataStructures();

  /* 機体の全質量． */
  const double& mass() const;

  /* 空気効力定数と回転数の積の和． */
  double dragRotorSum(const std::vector<double>& rot_speeds) const;

  /* 回転数から合計推力を求める． */
  double thrustSum(const std::vector<double>& rot_speeds);

private:
  const tobas::Drone& drone_;

  KDL::TreeFkSolverPos fk_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;
  tobas::RotorAxisExtractor z_rotors_;

  double mass_;
};
}  // namespace tobas_mr_common
