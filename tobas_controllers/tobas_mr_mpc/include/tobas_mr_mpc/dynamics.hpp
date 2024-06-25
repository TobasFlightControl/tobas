#pragma once

#include <Eigen/Core>

#include <tobas_linear_control/state_spaces.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

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
  void update(const double& roll, const double& pitch, const kdl::JntArray& q);

private:
  const tobas::Drone& drone_;

  kdl::TreeFkSolverPos fk_solver_;
  kdl::TreeJntToInertiaSolver inertia_solver_;
  tobas::RotorAxisExtractor z_rotors_;
};
}  // namespace tobas_mr_mpc
