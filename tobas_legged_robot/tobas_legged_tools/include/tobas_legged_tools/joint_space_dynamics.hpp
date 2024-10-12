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
  double friction_coef;  // [-] 静止摩擦係数
  double foot_diameter;  // [m] 足の接地面の直径

  double min_normal_force;  // [N]
  double max_normal_force;  // [N]

  double force_weight;  // 地面反力の参照値からのエラーに対するペナルティ
  double base_weight;   // 浮遊リンクの加速度の参照値からのエラーに対するペナルティ
};

/* 浮遊リンクの加速度を実現するための地面反力と関節トルクを求める (memo: 2-70) */
class JointSpaceDynamics
{
  static constexpr size_t kPosIdx = 0;
  static constexpr size_t kYawIdx = 3;
  static constexpr size_t kPitchIdx = 4;
  static constexpr size_t kRollIdx = 5;

  static constexpr size_t kBaseDoF = 6;     // 浮遊リンクの自由度
  static constexpr size_t kIneqSize = 8;    // 足1本あたりの不等式制約の個数
  static constexpr size_t kForceSize = 3;   // fx, fy, fz
  static constexpr size_t kTorqueSize = 1;  // tz
  static constexpr size_t kWrenchSize = kForceSize + kTorqueSize;

  static constexpr double kYawAngle = 0.;  // フットプリント座標系で考えるため，ヨー角は常にゼロ．
  static constexpr double kStandLegNormalForceThresh = 1.;  // [N] 目標垂直抗力がこれ以下なら遊脚と判定

public:
  explicit JointSpaceDynamics(
    const kdl::Tree& tree,
    const std::vector<std::string>& foot_names,
    const std::string& floating_base_name = "");

  void updateInternalDataStructures();

  bool configure(const JointSpaceDynamicsConfig& cfg);

  bool solve(
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
    const std::vector<double>& tar_torque);

  inline kdl::Vector getFootForce(size_t leg) const;
  inline double getFootTorque(size_t leg) const;
  inline kdl::JntArray getEffort() const;

  inline const std::string& errorMessage() const;

private:
  const kdl::Tree& tree_raw_;
  const std::vector<std::string> foot_names_;
  const std::string floating_base_name_;
  const size_t nc_, wrench_size_;

  // Config
  double friction_coef_;
  tobas_std::Range<double> normal_force_range_;

  Eigen::VectorXd w_out_;    // size = 3 * nc_
  Eigen::VectorXd eff_out_;  // size = kBaseDoF + nj_raw_
  std::string error_msg_;

  kdl::Tree tree_;  // 浮遊リンク付きのツリー
  size_t nj_raw_, nj_;
  kdl::JntArray cur_q_, cur_qd_, tar_qdd_;
  Eigen::MatrixXd J_;                                  // 足先位置のヤコビアンを並べたもの
  Eigen::VectorXd w_ref_;                              // 地面反力の参照値
  Eigen::Matrix<double, kIneqSize, kWrenchSize> A1_;   // 各足の不等式行列方程式の左辺
  Eigen::Matrix<double, kIneqSize, 1> b1_st_, b1_sw_;  // 各足の不等式行列方程式の右辺
  quadprog::DualActiveSetSolver qp_;

  kdl::TreeJntToJacSolver jac_solver_;
  kdl::TreeIdSolver_RNE rne_;
  kdl::TreeJntSpaceInertiaSolver mass_solver_;
  kdl::TreeJntToInertiaSolver inertia_solver_;
  kdl::TreeBoundingBoxSolver bb_solver_;

  double calcMass();
  double calcSizeScale();

  inline size_t forceIndex(const size_t& leg) const;
  inline size_t torqueIndex(const size_t& leg) const;
};

inline kdl::Vector JointSpaceDynamics::getFootForce(size_t leg) const
{
  return kdl::Vector(w_out_.segment<kForceSize>(forceIndex(leg)));
}

inline double JointSpaceDynamics::getFootTorque(size_t leg) const
{
  return w_out_(torqueIndex(leg));
}

inline kdl::JntArray JointSpaceDynamics::getEffort() const
{
  return kdl::JntArray(eff_out_.tail(nj_raw_));
}

inline size_t JointSpaceDynamics::forceIndex(const size_t& leg) const
{
  return kWrenchSize * leg;
}

inline size_t JointSpaceDynamics::torqueIndex(const size_t& leg) const
{
  return kWrenchSize * leg + kForceSize;
}

inline const std::string& JointSpaceDynamics::errorMessage() const
{
  return error_msg_;
}
}  // namespace lr_tools
