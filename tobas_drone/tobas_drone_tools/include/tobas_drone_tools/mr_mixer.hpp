#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

#include "./rotor_axis_extractor.hpp"

namespace tobas
{
/**
 * @brief 制約を考慮したマルチコプターの推力ミキシング (memo: 3-1)
 */
class MultiRotorMixer
{
public:
  explicit MultiRotorMixer(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures();

  bool solve(
    const double& cur_voltage,
    const kdl::JntArray& cur_q,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_dgyro_B,
    const double& tar_thrusts_sum,
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  const Eigen::VectorXd& getThrusts() const;

  bool setBaseWeight(double p);
  bool setThrustWeight(double p);

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  double base_weight_ = 1.;
  double thrust_weight_ = 1e-6;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;
  RotorAxisExtractor z_rotors_;

  quadprog::DualActiveSetSolver qp_;  // QPソルバー
  Eigen::Diagonal3d Q_;               // EoMの重み
  Eigen::DiagonalXd R_;               // 推力の重み
  Eigen::Matrix3Xd G_;                // EoM行列等式の左辺
  Eigen::Vector3d h_;                 // EoM行列等式の右辺

  void resizeAndFill();
  void updateWeight();
};
}  // namespace tobas
