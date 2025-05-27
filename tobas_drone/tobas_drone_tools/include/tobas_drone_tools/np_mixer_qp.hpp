#pragma once

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/tree_fk_solver_pos_all.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_quadprog/dual_active_set.hpp>

#include "./mixer.hpp"

namespace tobas
{
/**
 * @brief 制約を考慮した非平面配置マルチコプターの推力ミキシング (memo: 2-49)
 */
class NonPlanarMixer_QP : public Mixer
{
  using super = Mixer;

public:
  explicit NonPlanarMixer_QP(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  bool solve(
    const kdl::JntArray& cur_q,
    const kdl::Rotation& cur_rot,
    const kdl::Vector& cur_gyro_B,
    const kdl::Vector& tar_acc_W,
    const kdl::Vector& tar_dgyro_B,
    const kdl::Vector& ext_force_W = kdl::Vector::Zero(),
    const kdl::Vector& ext_torque_B = kdl::Vector::Zero());

  const Eigen::VectorXd& getThrusts() const;

  bool setLinearWeight(double p);
  bool setAngularWeight(double p);
  bool setThrustWeight(double p);

private:
  struct Config
  {
    double linear_weight = 1.;
    double angular_weight = 1.;
    double thrust_weight = 1e-9;
  } cfg_;

  kdl::TreeFkSolverPosAll fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  quadprog::DualActiveSetSolver qp_;  // QPソルバー
  Eigen::Diagonal6d Q_;               // EoMの重み
  Eigen::DiagonalXd R_;               // 推力の重み
  Eigen::Matrix6Xd G_;                // EoM行列等式の左辺
  Eigen::Vector6d h_;                 // EoM行列等式の右辺

  void resizeAndFill();
};
}  // namespace tobas
