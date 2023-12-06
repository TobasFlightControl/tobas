#pragma once

#include <Eigen/Core>

#include <dh_linear_control/state_spaces.hpp>
#include <dh_kdl/treefksolverpos.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>

#include <tobas_tools/drone.hpp>
#include <tobas_tools/rotor_axis_extractor.hpp>

namespace tobas_mr_mpc
{
/**
 * @brief クアッドロータの連続時間状態方程式．
 */
class MultiRotorDynamics : public ctrl::LinearDynamics
{
public:
  explicit MultiRotorDynamics(const tobas::Drone& drone);

  void updateInternalDataStructures();

  /**
   * @brief 状態方程式を更新する．
   *
   * @param roll {world}に対する{base}のロール角
   * @param pitch {world}に対する{base}のピッチ角
   * @param q 関節角
   */
  void update(const double& roll, const double& pitch, const KDL::JntArray& q);

private:
  const tobas::Drone& drone_;

  KDL::TreeFkSolverPos fk_solver_;
  KDL::TreeJntToInertiaSolver inertia_solver_;
  tobas::RotorAxisExtractor z_rotors_;

  KDL::Frame T_base_rotor_;
  KDL::RigidBodyInertia I_base_;
};
}  // namespace tobas_mr_mpc
