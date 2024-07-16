#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "../include/tobas_legged_tools/joint_space_dynamics.hpp"

using namespace std;
using namespace Eigen;
namespace et = eigen_tools;

namespace lr_tools
{
JointSpaceDynamics::JointSpaceDynamics(
  const kdl::Tree& tree,
  const vector<string>& foot_names,
  const string& floating_base_name)
  : tree_raw_(tree),
    foot_names_(foot_names),
    floating_base_name_(floating_base_name),
    nc_(foot_names.size()),
    force_size_(3 * nc_),
    f_ref_(force_size_),
    jac_solver_(tree_),
    rne_(tree_),
    mass_solver_(tree_),
    inertia_solver_(tree_),
    bb_solver_(tree_)
{
  CI_part_.setZero();
  CI_part_(0, 2) = -1;
  CI_part_(1, 2) = 1;
  CI_part_(2, 0) = -1;
  CI_part_(3, 0) = 1;
  CI_part_(4, 1) = -1;
  CI_part_(5, 1) = 1;

  ci0_part_stand_.setZero();
  ci0_part_swing_.setZero();  // 遊脚の反力の等式制約は反力の範囲を0にすることで表現

  qp_.resize(force_size_ + kBaseDoF, kBaseDoF, kIneqSizePerLeg * nc_);
  qp_.setZero();

  updateInternalDataStructures();
}

void JointSpaceDynamics::updateInternalDataStructures()
{
  // ツリーに浮遊リンクを接続
  tree_ = kdl::Tree::FloatingBase("world", floating_base_name_);
  tree_.addTree(tree_raw_, floating_base_name_);

  nj_raw_ = tree_raw_.getNrOfJoints();
  nj_ = tree_.getNrOfJoints();
  J_.resize(force_size_, nj_);

  jac_solver_.updateInternalDataStructures();
  rne_.updateInternalDataStructures();
  mass_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  bb_solver_.updateInternalDataStructures();

  cur_q_.resize(nj_);
  cur_qd_.resize(nj_);
  tar_qdd_.resize(nj_);

  qp_.x_scale.head(force_size_).fill(calcMass() * tobas_std::kGravity / nc_);
  qp_.x_scale.segment<3>(force_size_).fill(sqrt(tobas_std::kGravity * calcSizeScale()));  // フルード数に基づく
  qp_.x_scale.segment<3>(force_size_ + 3).fill(M_PI);
}

bool JointSpaceDynamics::configure(const JointSpaceDynamicsConfig& cfg)
{
  // TODO: 有効な値かチェック

  CI_part_.block<4, 1>(2, 2).fill(-cfg.friction_coef);
  const MatrixXd CI_left = et::blockDiag(CI_part_, nc_);
  const MatrixXd CI_right = MatrixXd::Zero(kIneqSizePerLeg * nc_, kBaseDoF);
  qp_.problem.A = et::concat(CI_left, CI_right, 1);

  ci0_part_stand_(0) = -cfg.min_normal_force;
  ci0_part_stand_(1) = cfg.max_normal_force;

  qp_.problem.P.diagonal().head(force_size_).fill(cfg.force_weight);
  qp_.problem.P.diagonal().tail<kBaseDoF>().fill(cfg.base_weight);

  return true;
}

void JointSpaceDynamics::solve(
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
  const vector<bool>& is_stand)
{
  assert(cur_q.size() == nj_raw_);
  assert(cur_qd.size() == nj_raw_);
  assert(tar_qdd.size() == nj_raw_);
  assert(tar_force.size() == nc_);
  assert(is_stand.size() == nc_);

  // 浮遊リンクを含む関節状態の現在値を更新
  cur_q_.data.segment<3>(kPosIdx).setZero();  // TODO: Z座標を入れなくても問題ない？
  cur_q_(kRollIdx) = roll;
  cur_q_(kPitchIdx) = pitch;
  cur_q_(kYawIdx) = kYawAngle;
  cur_qd_.data.segment<3>(kPosIdx) = cur_vel.data;
  cur_qd_.data.segment<3>(kYawIdx) = et::eulerrateFromAngvelGlobal(cur_gyro.data, pitch, kYawAngle).reverse();
  cur_q_.data.tail(nj_raw_) = cur_q.data;
  cur_qd_.data.tail(nj_raw_) = cur_qd.data;

  // ヤコビアンを更新
  for (size_t l = 0; l < nc_; ++l)
  {
    if (jac_solver_.JntToJac(cur_q_, foot_names_[l]) < 0)
      throw runtime_error("Jacobian solver failed: " + jac_solver_.errorMessage());
    J_.block(3 * l, 0, 3, nj_) = jac_solver_.getJacobian().data.topRows<3>();  // 並進部分のみ取り出す
  }

  // 浮遊リンクを含む関節角速度の目標値を更新
  tar_qdd_.data.segment<3>(kPosIdx) = tar_acc.data;
  tar_qdd_.data(kRollIdx) = tar_rpydd.roll;
  tar_qdd_.data(kPitchIdx) = tar_rpydd.pitch;
  tar_qdd_.data(kYawIdx) = tar_rpydd.yaw;
  tar_qdd_.data.tail(nj_raw_) = tar_qdd.data;

  // 地面反力の参照値を更新
  for (size_t l = 0; l < nc_; ++l)
    f_ref_.segment<3>(3 * l) = tar_force[l].data;

  // 外力項を除いたトルクを計算
  if (rne_.CartToJnt(cur_q_, cur_qd_, tar_qdd_) < 0)
    throw runtime_error("RNE failed: " + rne_.errorMessage());

  // QPPを作成
  const Matrix<double, kBaseDoF, Dynamic> Jt_base = J_.leftCols<kBaseDoF>().transpose();
  if (mass_solver_.JntToMass(cur_q_) < 0)
    throw runtime_error("Mass solver failed: " + mass_solver_.errorMessage());
  const Matrix<double, kBaseDoF, kBaseDoF> Mb = mass_solver_.getMass().data.topLeftCorner<kBaseDoF, kBaseDoF>();

  qp_.problem.G.leftCols(force_size_) = Jt_base;
  qp_.problem.G.rightCols<kBaseDoF>() = -Mb;

  qp_.problem.h = rne_.getEfforts().data.head<kBaseDoF>() - Jt_base * f_ref_;

  for (size_t l = 0; l < nc_; ++l)
  {
    if (is_stand[l])
      qp_.problem.b.segment<kIneqSizePerLeg>(kIneqSizePerLeg * l) =
        ci0_part_stand_ - CI_part_ * f_ref_.segment<3>(3 * l);
    else
      qp_.problem.b.segment<kIneqSizePerLeg>(kIneqSizePerLeg * l) =
        ci0_part_swing_ - CI_part_ * f_ref_.segment<3>(3 * l);
  }

  // QPPを解く
  const VectorXd x = qp_.solve();

  // 地面反力と浮遊リンクの加速度の残渣を取得
  const VectorXd f_res = x.head(force_size_);
  const VectorXd qdd_res = x.tail<kBaseDoF>();

  // 修正された地面反力と関節トルクを計算
  f_out_ = f_ref_ + x.head(force_size_);
  eff_out_ = rne_.getEfforts().data - J_.transpose() * f_out_;
  eff_out_.head<kBaseDoF>() += Mb * qdd_res;                                       // ベースの修正分
  assert(et::isClose(eff_out_.head<kBaseDoF>().eval(), Vector6d::Zero().eval()));  // ベースのレンチは0になるはず
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
