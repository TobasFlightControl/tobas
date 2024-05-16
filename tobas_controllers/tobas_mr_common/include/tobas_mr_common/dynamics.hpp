#pragma once

#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

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
  tobas_kdl::Vector relativePerpVel(
    const tobas_kdl::Rotation& rot,
    const tobas_kdl::Vector& vel_B,
    const tobas_kdl::Vector& wind_W);

  /* 機体座標系から見たH-force */
  tobas_kdl::Vector horizontalForce(
    const tobas_kdl::Rotation& rot,
    const tobas_kdl::Vector& vel_B,
    const tobas_kdl::Vector& wind_W,
    const std::vector<double>& rot_speeds);

  /* 機体座標系から見たH-forceによるモーメント (memo: 2-34) */
  tobas_kdl::Vector horizontalMoment(
    const tobas_kdl::Rotation& rot,
    const tobas_kdl::Vector& vel_B,
    const tobas_kdl::Vector& wind_W,
    const tobas_kdl::JntArray& q,
    const std::vector<double>& rot_speeds);

private:
  const tobas::Drone& drone_;

  tobas_kdl::TreeFkSolverPos fk_solver_;
  tobas_kdl::TreeJntToInertiaSolver inertia_solver_;
  tobas::RotorAxisExtractor z_rotors_;
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
}  // namespace tobas_mr_common
