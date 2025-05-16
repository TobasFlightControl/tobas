#include "tobas_legged_tools/joint_space_dynamics.hpp"

#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;
using namespace Eigen;

namespace lr_tools
{
JointSpaceDynamics::JointSpaceDynamics(
  const kdl::Tree& tree,
  const vector<string>& foot_names,
  const string& floating_base_name)
  : tree_raw_(tree)
  , foot_names_(foot_names)
  , floating_base_name_(floating_base_name)
  , nc_(foot_names.size())
  , wrench_size_(kWrenchSize * nc_)
  , w_ref_(wrench_size_)
  , jac_solver_(tree_)
  , rne_(tree_)
  , mass_solver_(tree_)
  , inertia_solver_(tree_)
  , bb_solver_(tree_)
{
  A1_.setZero();
  A1_(0, 2) = -1;
  A1_(1, 2) = 1;
  A1_(2, 0) = -1;
  A1_(3, 0) = 1;
  A1_(4, 1) = -1;
  A1_(5, 1) = 1;
  A1_(6, 3) = -1;
  A1_(7, 3) = 1;

  b1_st_.setZero();
  b1_sw_.setZero();  // 遊脚の反力の等式制約は反力の範囲を0にすることで表現

  qp_.resize(wrench_size_ + kBaseDoF, kBaseDoF, kIneqSize * nc_);
  qp_.setZero();

  if (tree.getNrOfJoints() > 0) {
    if (!updateInternalDataStructures()) {
      PRINT_ERROR("Failed to update internal data structures of lr_tools::JointSpaceDynamics.");
    }
  }
}

bool JointSpaceDynamics::updateInternalDataStructures()
{
  // 浮遊リンクの名前を取得
  auto floating_base_name = floating_base_name_;
  if (floating_base_name.empty()) {
    floating_base_name = tree_raw_.getRootName();
  }

  // ベースリンク以下を抽出
  kdl::Tree base_sub_tree;
  if (!tree_raw_.getSubTree(floating_base_name, base_sub_tree)) {
    cerr << "Failed to get sub tree." << endl;
    return false;
  }

  // ツリーに浮遊リンクを接続
  tree_ = kdl::Tree::FloatingBase("world", floating_base_name);
  if (!tree_.addTree(base_sub_tree, floating_base_name)) {
    cerr << "Failed to add a floating base link to the tree." << endl;
    return false;
  }

  nj_raw_ = base_sub_tree.getNrOfJoints();
  nj_ = tree_.getNrOfJoints();
  J_.resize(wrench_size_, nj_);

  if (!jac_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!rne_.updateInternalDataStructures()) {
    return false;
  }
  if (!mass_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!inertia_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!bb_solver_.updateInternalDataStructures()) {
    return false;
  }

  cur_q_.resize(nj_);
  cur_qd_.resize(nj_);
  tar_qdd_.resize(nj_);

  qp_.x_scale.head(wrench_size_).fill(calcMass() * tobas_std::kGravity / nc_);  // TODO: 力とトルクでスケールを分ける
  qp_.x_scale.segment<3>(wrench_size_).fill(sqrt(tobas_std::kGravity * calcSizeScale()));  // フルード数に基づく
  qp_.x_scale.segment<3>(wrench_size_ + 3).fill(M_PI);

  return true;
}

bool JointSpaceDynamics::configure(const JointSpaceDynamicsConfig& cfg)
{
  // TODO: 有効な値かチェック

  A1_.block<4, 1>(2, 2).fill(-cfg.friction_coef);
  A1_.block<2, 1>(6, 2).fill(-cfg.friction_coef * cfg.foot_diameter);
  const MatrixXd CI_left = eigen::blockDiag(A1_, nc_);
  const MatrixXd CI_right = MatrixXd::Zero(kIneqSize * nc_, kBaseDoF);
  qp_.problem.A = eigen::concat(CI_left, CI_right, 1);

  b1_st_(0) = -cfg.min_normal_force;
  b1_st_(1) = cfg.max_normal_force;

  qp_.problem.P.diagonal().head(wrench_size_).fill(cfg.force_weight);
  qp_.problem.P.diagonal().tail<kBaseDoF>().fill(cfg.base_weight);

  return true;
}

bool JointSpaceDynamics::solve(
  const double& roll,
  const double& pitch,
  const kdl::Vector& cur_vel,
  const kdl::Vector& cur_gyro,
  const kdl::JntArray& cur_q,
  const kdl::JntArray& cur_qd,
  const kdl::Vector& tar_acc,
  const kdl::Euler& tar_rpydd,
  const kdl::JntArray& tar_qdd,
  const vector<kdl::Vector>& tar_force,
  const vector<double>& tar_torque)
{
  assert(cur_q.size() == nj_raw_);
  assert(cur_qd.size() == nj_raw_);
  assert(tar_qdd.size() == nj_raw_);
  assert(tar_force.size() == nc_);
  assert(tar_torque.size() == nc_);

  // 浮遊リンクを含む関節状態の現在値を更新
  cur_q_.data.segment<3>(kPosIdx).setZero();  // 並進位置は関係ない
  cur_q_(kRollIdx) = roll;
  cur_q_(kPitchIdx) = pitch;
  cur_q_(kYawIdx) = kYawAngle;
  cur_qd_.data.segment<3>(kPosIdx) = cur_vel.data;
  cur_qd_.data.segment<3>(kYawIdx) = eigen::eulerrateFromAngvelGlobal(cur_gyro.data, pitch, kYawAngle).reverse();
  cur_q_.data.tail(nj_raw_) = cur_q.data;
  cur_qd_.data.tail(nj_raw_) = cur_qd.data;

  // ヤコビアンを更新
  for (size_t l = 0; l < nc_; ++l) {
    if (jac_solver_.JntToJac(cur_q_, foot_names_[l]) < 0) {
      error_msg_ = "Jacobian solver failed: " + jac_solver_.errorMessage();
      return false;
    }
    const auto& jac = jac_solver_.getJacobian().data;
    J_.block(forceIndex(l), 0, kForceSize, nj_) = jac.topRows<kForceSize>();  // vx, vy, vz
    J_.block(torqueIndex(l), 0, 1, nj_) = jac.row(5);                         // wz
  }

  // 浮遊リンクを含む関節角速度の目標値を更新
  tar_qdd_.data.segment<3>(kPosIdx) = tar_acc.data;
  tar_qdd_.data(kRollIdx) = tar_rpydd.roll;
  tar_qdd_.data(kPitchIdx) = tar_rpydd.pitch;
  tar_qdd_.data(kYawIdx) = tar_rpydd.yaw;
  tar_qdd_.data.tail(nj_raw_) = tar_qdd.data;

  // 地面反力の参照値を更新
  for (size_t l = 0; l < nc_; ++l) {
    w_ref_.segment<kForceSize>(forceIndex(l)) = tar_force[l].data;
    w_ref_(torqueIndex(l)) = tar_torque[l];
  }

  // 外力項を除いたトルクを計算
  if (rne_.CartToJnt(cur_q_, cur_qd_, tar_qdd_) < 0) {
    error_msg_ = "RNE failed: " + rne_.errorMessage();
    return false;
  }

  // 質量行列を計算
  if (mass_solver_.JntToMass(cur_q_) < 0) {
    error_msg_ = "Mass solver failed: " + mass_solver_.errorMessage();
    return false;
  }

  // QPPを作成
  const Matrix<double, kBaseDoF, Dynamic> Jt_base = J_.leftCols<kBaseDoF>().transpose();
  const Matrix<double, kBaseDoF, kBaseDoF> Mb = mass_solver_.getMass().data.topLeftCorner<kBaseDoF, kBaseDoF>();

  qp_.problem.G.leftCols(wrench_size_) = Jt_base;
  qp_.problem.G.rightCols<kBaseDoF>() = -Mb;

  qp_.problem.h = rne_.getEfforts().data.head<kBaseDoF>() - Jt_base * w_ref_;

  for (size_t l = 0; l < nc_; ++l) {
    // 各足の接地状態を制御対象として扱うため，目標地面反力から接地状態を決定する．
    const auto is_stand = tar_force[l].z() > kStandLegNormalForceThresh;
    if (is_stand) {
      qp_.problem.b.segment<kIneqSize>(kIneqSize * l) = b1_st_ - A1_ * w_ref_.segment<kWrenchSize>(forceIndex(l));
    }
    else {
      qp_.problem.b.segment<kIneqSize>(kIneqSize * l) = b1_sw_ - A1_ * w_ref_.segment<kWrenchSize>(forceIndex(l));
    }
  }

  // QPPを解く
  if (!qp_.solve()) {
    error_msg_ = "QP failed: " + qp_.errorMessage();
    return false;
  }

  // 地面反力と浮遊リンクの加速度の残渣を取得
  const VectorXd w_res = qp_.solution().head(wrench_size_);
  const VectorXd qdd_res = qp_.solution().tail<kBaseDoF>();

  // 修正された地面反力と関節トルクを計算
  w_out_ = w_ref_ + w_res;
  eff_out_ = rne_.getEfforts().data - J_.transpose() * w_out_;
  eff_out_.head<kBaseDoF>() += Mb * qdd_res;                                          // ベースの修正分
  assert(eigen::isClose(eff_out_.head<kBaseDoF>().eval(), Vector6d::Zero().eval()));  // ベースのレンチは0になるはず

  return true;
}

double JointSpaceDynamics::calcMass()
{
  inertia_solver_.JntToCart(kdl::JntArray::Zero(nj_));
  return inertia_solver_.getInertia().getMass();
}

double JointSpaceDynamics::calcSizeScale()
{
  bb_solver_.solve(kdl::JntArray::Zero(nj_));
  return bb_solver_.diagonalLength();
}
}  // namespace lr_tools
