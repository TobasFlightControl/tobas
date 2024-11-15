#pragma once

#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

#include "./rotor_axis_extractor.hpp"

namespace tobas
{
/* 平面配置マルチコプターの運動方程式の要素． */
class MultirotorDynamicsComponents
{
public:
  explicit MultirotorDynamicsComponents(const Drone& drone, const kdl::Tree& tree);

  void updateInternalDataStructures();

  /* 機体の全質量． */
  inline const double& mass() const;

  /* 回転数から合計推力を求める． */
  inline double thrustSum(const std::vector<double>& rot_speeds);

  /* 最大推力の合計． */
  inline double maxThrustSum(const double& battery_voltage) const;

  /* 最小推力の合計． */
  inline double minThrustSum(const double& battery_voltage) const;

  /* 空気効力定数と回転数の積の和． */
  double dragRotorSum(const std::vector<double>& rot_speeds) const;

  /* 機体の風に対する相対速度のプロペラに対する水平成分を求める．機体座標系ではZ成分のみ0としたベクトルに等しい．*/
  kdl::Vector relativePerpVel(const kdl::Rotation& rot, const kdl::Vector& vel_B, const kdl::Vector& wind_W);

  /* 機体座標系から見たH-force */
  kdl::Vector horizontalForce(
    const kdl::Rotation& rot,
    const kdl::Vector& vel_B,
    const kdl::Vector& wind_W,
    const std::vector<double>& rot_speeds);

  /* 機体座標系から見たH-forceによるモーメント (memo: 2-34) */
  kdl::Vector horizontalMoment(
    const kdl::Rotation& rot,
    const kdl::Vector& vel_B,
    const kdl::Vector& wind_W,
    const kdl::JntArray& q,
    const std::vector<double>& rot_speeds);

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeFkSolverPos fk_solver_;
  kdl::TreeJntToInertiaSolver inertia_solver_;
  RotorAxisExtractor z_rotors_;
};

inline const double& MultirotorDynamicsComponents::mass() const
{
  return inertia_solver_.getInertia().getMass();
}

inline double MultirotorDynamicsComponents::thrustSum(const std::vector<double>& rot_speeds)
{
  return z_rotors_.thrustSum(rot_speeds);
}

inline double MultirotorDynamicsComponents::maxThrustSum(const double& battery_voltage) const
{
  return z_rotors_.maxThrustSum(battery_voltage);
}

inline double MultirotorDynamicsComponents::minThrustSum(const double& battery_voltage) const
{
  return z_rotors_.minThrustSum(battery_voltage);
}
}  // namespace tobas
