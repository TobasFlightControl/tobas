#include <tobas_std_tools/universal_constants.hpp>

#include "../include/tobas_legged_tools/state_estimator.hpp"

#define EPS 1e-9

using namespace std;
using namespace Eigen;

namespace lr_tools
{
StateEstimator::StateEstimator(const kdl::Tree& tree, const vector<string>& foot_names)
  : foot_names_(foot_names),
    nc_(foot_names.size()),
    fk_solver_(tree),
    cont_(tree, foot_names),
    kf_(cont_.stateSize(), cont_.inputSize(), 6 + 4 * nc_, cont_.stateSize()),
    c2d_(cont_.stateSize(), cont_.inputSize())
{
  updateInternalDataStructures();
}

void StateEstimator::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  cont_.updateInternalDataStructures();

  initializeKalmanFilter();
}

bool StateEstimator::configure(const StateEstimatorConfig& cfg)
{
  if (cfg.variance_coef <= 0)
    return false;

  cfg_ = cfg;
  return true;
}

void StateEstimator::update(
  const kdl::Quaternion& W_Quat_B,
  const kdl::Vector& gyro_B,
  const kdl::JntArray& q,
  const kdl::JntArray& qd,
  const vector<bool>& is_stand,
  const vector<double>& contact_probs,
  const vector<kdl::Vector>& foot_forces,
  const vector<double>& foot_torques,
  const double& dt)
{
  assert(is_stand.size() == nc_);
  assert(contact_probs.size() == nc_);
  assert(foot_forces.size() == nc_);
  assert(foot_forces.size() == nc_);
  assert(dt >= 0);

  /* ===== 事前計算 ===== */
  W_Quat_B.getRPY(roll_, pitch_, yaw_);
  const auto FP_Rot_B = kdl::Rotation::RPY(roll_, pitch_, 0);
  const auto gyro_FP = FP_Rot_B * gyro_B;

  /* ===== ダイナミクスを更新 ===== */
  cont_.update(roll_, pitch_, q, is_stand);
  const auto disc_dyn = c2d_.convert(cont_, dt);
  kf_.ss.updateDynamics(disc_dyn);

  /* ===== 観測ノイズの共分散行列を更新 ===== */
  // オイラー角
  kf_.R(kRollIdx, kRollIdx) = EPS;
  kf_.R(kPitchIdx, kPitchIdx) = EPS;

  // 回転速度
  kf_.R.diagonal().segment<3>(kGyroIdx).fill(EPS);

  // 重力
  kf_.R(kGravIdx, kGravIdx) = EPS;

  for (size_t l = 0; l < nc_; ++l)
  {
    // 分散を計算
    double var;
    if (is_stand[l])
      var = max(cfg_.variance_coef * pow(1 - contact_probs[l], cfg_.variance_exp), EPS);
    else
      var = numeric_limits<double>::max();

    // 地面からの高さ
    kf_.R(altIdx(l), altIdx(l)) = var;

    // 並進速度
    kf_.R.diagonal().segment<3>(velIdx(l)).fill(var);
  }

  /* ===== 出力ベクトルを更新 ===== */
  // オイラー角
  kf_.y(kRollIdx) = roll_;
  kf_.y(kPitchIdx) = pitch_;

  // 回転速度
  kf_.y.segment<3>(kGyroIdx) = gyro_FP.data;

  // 重力
  kf_.y(kGravIdx) = tobas_std::kGravity;

  for (size_t l = 0; l < nc_; ++l)
  {
    if (is_stand[l])
    {
      // 順運動学
      if (fk_solver_.JntToCart(q, qd, foot_names_[l]) < 0)
        throw runtime_error("FK failed: " + fk_solver_.errorMessage());
      const auto& foot_pos = fk_solver_.getFrameVel().p.p;
      const auto& foot_vel = fk_solver_.getFrameVel().p.v;

      // 地面からの高さを立脚の場合は足先のz座標を符号反転したもので推定
      // 定常誤差があるので単純にオフセットを設ける
      const auto foot_height = (FP_Rot_B * foot_pos).z();
      kf_.y(altIdx(l)) = -foot_height + kFootGroundOffset;

      // 並進速度 (memo: 1-28)
      kf_.y.segment<3>(velIdx(l)) = -(FP_Rot_B * (gyro_B * foot_pos + foot_vel)).data;
    }
    else
    {
      // 遊脚の場合は予測状態をそのまま観測状態とする
      // TODO: もっと良い推定方法を考える
      kf_.y(altIdx(l)) = kf_.state()(LinearDynamics::kAltIdx);
      kf_.y.segment<3>(velIdx(l)) = kf_.state().segment<3>(LinearDynamics::kVelXIdx);
    }
  }

  /* ===== 制御入力を更新 ===== */
  // FIXME: 状態推定が制御入力に依存すると発散のリスクがあるため，
  for (size_t l = 0; l < nc_; ++l)
  {
    kf_.u.segment<3>(cont_.forceIndex(l)) = foot_forces[l].data;
    kf_.u(cont_.torqueIndex(l)) = foot_torques[l];
  }

  /* ===== カルマンフィルタを1ステップ進める ===== */
  kf_.update();
}

void StateEstimator::initializeKalmanFilter()
{
  kf_.setZero();

  kf_.ss.C = makeCy();
  kf_.Bv.diagonal().setOnes();  // システム雑音は直接加わるとする
  kf_.Q.diagonal().fill(EPS);

  VectorXd init_x(cont_.stateSize());
  init_x << 0, 0, kInitTrunkHeight, 0, 0, 0, 0, 0, 0, tobas_std::kGravity;  // FIXME: 胴体高さの初期値を推定
  kf_.initialize(init_x, MatrixXd::Identity(cont_.stateSize(), cont_.stateSize()));
}

MatrixXd StateEstimator::makeCy()
{
  MatrixXd Cy = MatrixXd::Zero(kf_.outputSize(), cont_.stateSize());

  // オイラー角
  Cy(kRollIdx, LinearDynamics::kRollIdx) = 1;
  Cy(kPitchIdx, LinearDynamics::kPitchIdx) = 1;

  // 回転速度
  Cy.block<3, 3>(kGyroIdx, LinearDynamics::kGyroXIdx).diagonal().setOnes();

  // 重力
  Cy(kGravIdx, LinearDynamics::kGravIdx) = 1;

  // 地面からの高さ
  for (size_t l = 0; l < nc_; ++l)
    Cy(altIdx(l), LinearDynamics::kAltIdx) = 1;

  // 並進速度
  for (size_t l = 0; l < nc_; ++l)
    Cy.block<3, 3>(velIdx(l), LinearDynamics::kVelXIdx).diagonal().setOnes();

  return Cy;
}
}  // namespace lr_tools
