#pragma once

#include <tobas_std_tools/range.hpp>
#include <tobas_quadprog/dual_active_set.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_kdl/jntspaceinertiamatrix.hpp>
#include <tobas_kdl/treejnttojacsolver.hpp>
#include <tobas_kdl/treeidsolver_rne.hpp>
#include <tobas_kdl/treejntspaceinertiasolver.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>
#include <tobas_kdl/treeboundingboxsolver.hpp>

namespace lr_tools
{
struct JointSpaceDynamicsConfig
{
  double friction_coef;  // [-]

  double min_normal_force;  // [N]
  double max_normal_force;  // [N]

  double force_weight;  // 地面反力の参照値からのエラーに対するペナルティ
  double base_weight;   // 浮遊リンクの加速度の参照値からのエラーに対するペナルティ
};

/* 浮遊リンクの加速度を実現するための地面反力を求める． */
class JointSpaceDynamics
{
  static constexpr size_t kPosIdx = 0;
  static constexpr size_t kYawIdx = 3;
  static constexpr size_t kPitchIdx = 4;
  static constexpr size_t kRollIdx = 5;

  static constexpr size_t kBaseDoF = 6;         // 浮遊リンクの自由度
  static constexpr size_t kIneqSizePerLeg = 6;  // 足1本あたりの不等式制約の個数

  static constexpr double kYawAngle = 0.;  // フットプリント座標系で考えるため，ヨー角は常にゼロ．

public:
  explicit JointSpaceDynamics(
    const kdl::Tree& tree,
    const std::vector<std::string>& foot_names,
    const std::string& floating_base_name);

  void updateInternalDataStructures();

  bool configure(const JointSpaceDynamicsConfig& cfg);

  void solve(
    const double& roll,
    const double& pitch,
    const kdl::Vector& cur_vel,
    const kdl::Vector& cur_gyro,
    const kdl::JntArray& cur_q,
    const kdl::JntArray& cur_qd,
    const kdl::Vector& tar_acc,
    const kdl::Euler& tar_rpydd,
    const kdl::JntArray& tar_qdd,
    const std::vector<kdl::Vector>& tar_force,
    const std::vector<bool>& is_stand);

  inline kdl::Vector getForce(size_t leg) const;
  inline kdl::JntArray getEffort() const;

private:
  const kdl::Tree& tree_raw_;
  const std::vector<std::string> foot_names_;
  const std::string floating_base_name_;
  const size_t nc_, force_size_;

  // Config
  double friction_coef_;
  tobas_std::Range<double> normal_force_range_;

  Eigen::VectorXd f_out_;    // size = 3 * nc_
  Eigen::VectorXd eff_out_;  // size = kBaseDoF + nj_raw_

  kdl::Tree tree_;  // 浮遊リンク付きのツリー
  size_t nj_raw_, nj_;
  kdl::JntArray cur_q_, cur_qd_, tar_qdd_;
  Eigen::MatrixXd J_;                                  // 足先位置のヤコビアンを並べたもの
  Eigen::VectorXd f_ref_;                              // 地面反力の参照値
  Eigen::Matrix<double, kIneqSizePerLeg, 3> CI_part_;  // (memo: 1-41)のA
  Eigen::Matrix<double, kIneqSizePerLeg, 1> ci0_part_stand_, ci0_part_swing_;
  quadprog::DualActiveSetSolver qp_;

  kdl::TreeJntToJacSolver jac_solver_;
  kdl::TreeIdSolver_RNE rne_;
  kdl::TreeJntSpaceInertiaSolver mass_solver_;
  kdl::TreeJntToInertiaSolver inertia_solver_;
  kdl::TreeBoundingBoxSolver bb_solver_;

  double calcMass();
  double calcSizeScale();
};

inline kdl::Vector JointSpaceDynamics::getForce(size_t leg) const
{
  return kdl::Vector(f_out_.segment<3>(3 * leg));
}

inline kdl::JntArray JointSpaceDynamics::getEffort() const
{
  return kdl::JntArray(eff_out_.tail(nj_raw_));
}
}  // namespace lr_tools
