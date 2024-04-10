#include <tobas_std_tools/math.hpp>
#include <tobas_std_tools/assert.hpp>
#include <tobas_std_tools/console.hpp>

#include <tobas_tools/constants.hpp>

#include "../include/state_estimation_eskf/eskf.hpp"

#define I3 Matrix3d::Identity()

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace et = eigen_tools;

namespace state_estimation_eskf
{
ErrorStateKalmanFilter::ErrorStateKalmanFilter()
{
  TOBAS_DEBUG("ErrorStateKalmanFilter::ErrorStateKalmanFilter");

  // 観測方程式の固定部分を埋める
  H_pos_.setZero();
  H_xy_.setZero();
  H_z_.setZero();
  H_vel_.setZero();
  H_pv_.setZero();
  H_theta_.setZero();
  H_acc_.setZero();
  H_mag_.setZero();

  H_pos_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_xy_.block<2, 2>(0, kDeltaPosIdx).diagonal().setOnes();
  H_z_(0, kDeltaAltIdx) = 1;
  H_vel_.block<3, 3>(0, kDeltaVelIdx).diagonal().setOnes();
  H_pv_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_pv_.block<3, 3>(3, kDeltaVelIdx).diagonal().setOnes();
  H_theta_.block<3, 3>(0, kDeltaThetaIdx).diagonal().setOnes();  // 回転の誤差を3Dベクトルとして観測
  H_acc_.block<3, 3>(0, kAccBiasIdx).diagonal().setOnes();
}

void ErrorStateKalmanFilter::initialize(
  const Vector3d& init_pos,
  const Vector3d& init_vel,
  const Quaterniond& init_quat,
  const Matrix3d& init_pos_cov,
  const Matrix3d& init_vel_cov,
  const Matrix3d& init_dtheta_cov,
  const Matrix3d& init_acc_bias_cov,
  const Matrix3d& init_gyro_bias_cov,
  const double& init_grav_var)
{
  TOBAS_DEBUG("ErrorStateKalmanFilter::initialize");

  assert(et::isSymmetricSemiPositiveDefinite(init_pos_cov));
  assert(et::isSymmetricSemiPositiveDefinite(init_vel_cov));
  assert(et::isSymmetricSemiPositiveDefinite(init_dtheta_cov));
  assert(et::isSymmetricSemiPositiveDefinite(init_acc_bias_cov));
  assert(et::isSymmetricSemiPositiveDefinite(init_gyro_bias_cov));
  assert(init_grav_var >= 0);

  // ノミナル状態を初期化
  x_.setZero();
  x_.segment<3>(kPosIdx) = init_pos;
  x_.segment<3>(kVelIdx) = init_vel;
  x_.segment<4>(kQuatIdx) = et::quaternionToHamilton(init_quat).normalized();
  x_(kGravIdx) = tobas::kGravity;

  // 共分散行列を初期化
  P_.setZero();
  P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx) = init_pos_cov;
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) = init_vel_cov;
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = init_dtheta_cov;
  P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx) = init_acc_bias_cov;
  P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx) = init_gyro_bias_cov;
  P_(kDeltaGravIdx, kDeltaGravIdx) = init_grav_var;

  // (270) ヤコビアンの不変部分を埋める
  F_x_.setIdentity();
}

void ErrorStateKalmanFilter::predictIMU(
  const Vector3d& acc_meas,
  const Vector3d& gyro_meas,
  const double& acc_noise_var,
  const double& gyro_noise_var,
  const double& acc_bias_noise_var,
  const double& gyro_bias_noise_var,
  const double& grav_var,
  const double& dt)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::predictIMU");

  assert(acc_noise_var >= 0);
  assert(gyro_noise_var >= 0);
  assert(acc_bias_noise_var >= 0);
  assert(gyro_bias_noise_var >= 0);
  assert(grav_var >= 0);
  assert(dt > 0);  // クオータニオンの正規化のためにdt = 0を許容できない

  const Matrix3d Rot = getDCM();
  const Vector3d acc_B = acc_meas - getAccelBias();
  const Vector3d acc_W = Rot * acc_B;
  const Vector3d delta_theta = (gyro_meas - getGyroBias()) * dt;
  const Quaterniond q_delta_theta = et::angleAxisToQuaternion(delta_theta);
  const Matrix3d R_delta_theta = q_delta_theta.toRotationMatrix();

  // (260) ノミナル状態のキネマティクス
  // x_.segment<3>(kPosIdx) += getVelocity() * dt + 0.5 * (acc_W + getGravVector()) * sqr(dt);
  x_.segment<3>(kPosIdx) += getVelocity() * dt;  // 積分誤差が大きくなるため二階積分は考えない
  x_.segment<3>(kVelIdx) += (acc_W + getGravVector()) * dt;
  x_.segment<4>(kQuatIdx) = et::quaternionToHamilton(getQuaternion() * q_delta_theta).normalized();

  // (270) ヤコビアンの可変部を更新
  F_x_.block<3, 3>(kDeltaPosIdx, kDeltaVelIdx).diagonal().fill(dt);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaThetaIdx) = -Rot * et::crossMat(acc_B) * dt;
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaAccBiasIdx) = -Rot * dt;
  F_x_(kDeltaVelIdx + 2, kDeltaGravIdx) = -dt;
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = R_delta_theta.transpose();
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaGyroBiasIdx).diagonal().fill(-dt);

  // (269)第一項: 共分散行列の予測値を更新
  // TODO: sympyを用いて行列積を効率化
  P_ = F_x_ * P_ * F_x_.transpose();
  et::symmetrise(P_);  // 対称化 (これが必須)

  // (269)第二項: プロセスノイズを印加
  P_.diagonal().segment<3>(kDeltaVelIdx).array() += acc_noise_var * sqr(dt);
  P_.diagonal().segment<3>(kDeltaThetaIdx).array() += gyro_noise_var * sqr(dt);
  P_.diagonal().segment<3>(kDeltaAccBiasIdx).array() += acc_bias_noise_var;
  P_.diagonal().segment<3>(kDeltaGyroBiasIdx).array() += gyro_bias_noise_var;
  P_(kDeltaGravIdx, kDeltaGravIdx) += grav_var;

  // NaN検出
  assertWithMsg(et::isFinite(x_), "Nominal state:" << x_.transpose());
  assertWithMsg(et::isFinite(F_x_), "F_x:\n" << F_x_);
  assertWithMsg(et::isFinite(P_), "Covariance matrix:\n" << P_);
}

double ErrorStateKalmanFilter::measurePosition(
  const Vector3d& pos_meas,
  const Matrix3d& pos_cov,
  const Vector3d& offset)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measurePosition");

  const Vector3d delta_pos = pos_meas - getPosition(offset);

  // 姿勢による偏微分
  const auto dqvq_dq = quatRotationDerivative(offset);
  const auto Q_dtheta = getQ_dtheta();
  H_pos_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  return correct<3>(delta_pos, pos_cov, H_pos_);
}

double ErrorStateKalmanFilter::measureXY(const Vector2d& xy_meas, const Matrix2d& xy_cov)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measureXY");

  const Vector2d delta_xy = xy_meas - getXY();
  return correct<2>(delta_xy, xy_cov, H_xy_);
}

double ErrorStateKalmanFilter::measureAltitude(const double& z_meas, const double& z_var)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measureAltitude");

  const double delta_z = z_meas - getAltitude();
  return correct<1>(Scalard(delta_z), Scalard(z_var), H_z_);
}

double ErrorStateKalmanFilter::measureVelocity(const Vector3d& vel_meas, const Matrix3d& vel_cov)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measureVelocity");

  return measureVelocity(vel_meas, vel_cov, Vector3d::Zero(), Vector3d::Zero());
}

double ErrorStateKalmanFilter::measureVelocity(
  const Vector3d& vel_meas,
  const Matrix3d& vel_cov,
  const Vector3d& offset,
  const Vector3d& gyro_meas)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measureVelocity");

  const Vector3d gyro_nominal = gyro_meas - getGyroBias();
  const Vector3d gyro_offset = gyro_nominal.cross(offset);
  const Vector3d vel_nominal = getVelocity() + getQuaternion() * gyro_offset;
  const Vector3d delta_vel = vel_meas - vel_nominal;

  // 姿勢による偏微分
  const auto dqvq_dq = quatRotationDerivative(gyro_offset);
  const auto Q_dtheta = getQ_dtheta();
  H_vel_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  // ジャイロバイアスによる偏微分
  H_vel_.block<3, 3>(0, kDeltaGyroBiasIdx) = getDCM() * et::crossMat(offset);

  return correct<3>(delta_vel, vel_cov, H_vel_);
}

double ErrorStateKalmanFilter::measurePosVel(
  const Vector3d& pos_meas,
  const Matrix3d& pos_cov,
  const Vector3d& vel_meas,
  const Matrix3d& vel_cov,
  const Vector3d& offset,
  const Vector3d& gyro_meas)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measurePosVel");

  // 観測誤差
  Vector6d delta;
  const Vector3d gyro_nominal = gyro_meas - getGyroBias();
  const Vector3d gyro_offset = gyro_nominal.cross(offset);
  const Vector3d vel_nominal = getVelocity() + getQuaternion() * gyro_offset;
  delta.head<3>() = pos_meas - getPosition(offset);  // 位置の誤差
  delta.tail<3>() = vel_meas - vel_nominal;          // 速度の誤差

  // 観測方程式
  const auto Q_dtheta = getQ_dtheta();
  const auto pos_q_deriv = quatRotationDerivative(offset);
  const auto vel_q_deriv = quatRotationDerivative(gyro_offset);
  H_pv_.block<3, 3>(0, kDeltaThetaIdx) = pos_q_deriv * Q_dtheta;  // 位置の姿勢による偏微分
  H_pv_.block<3, 3>(3, kDeltaThetaIdx) = vel_q_deriv * Q_dtheta;  // 速度の姿勢による偏微分
  H_pv_.block<3, 3>(0, kDeltaGyroBiasIdx) = getDCM() * et::crossMat(offset);

  // 共分散
  Matrix6d cov;
  cov.topLeftCorner<3, 3>() = pos_cov;
  cov.bottomRightCorner<3, 3>() = vel_cov;
  cov.topRightCorner<3, 3>().setZero();
  cov.bottomLeftCorner<3, 3>().setZero();

  // 事後推定を更新
  return correct<6>(delta, cov, H_pv_);
}

double
ErrorStateKalmanFilter::measureQuaternion(const Quaterniond& q_meas, const Matrix3d& theta_cov)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measureQuaternion");

  const Quaterniond q_nominal = getQuaternion();
  const Quaterniond q_error = q_nominal.conjugate() * q_meas;  // 回転の誤差
  const Vector3d delta_theta = et::quaternionToAngleAxis(q_error);

  return correct<3>(delta_theta, theta_cov, H_theta_);
}

double ErrorStateKalmanFilter::measureGravity(const Vector3d& acc_meas, const Matrix3d& grav_cov)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measureGravity");

  const Matrix3d R_B_W = getDCM().transpose();
  const Vector3d grav_B = R_B_W * getGravVector();
  const Vector3d acc_ref = getAccelBias() - grav_B;  // 動的な加速度なしで観測されるべき加速度
  const Vector3d delta_acc = acc_meas - acc_ref;

  H_acc_.block<3, 3>(0, kDeltaThetaIdx) = -2 * et::crossMat(grav_B);
  H_acc_.col(kDeltaGravIdx) = R_B_W.col(2);
  return correct<3>(delta_acc, grav_cov, H_acc_);
}

double ErrorStateKalmanFilter::measureYaw(const double& yaw_meas, const double& yaw_var)
{
  TOBAS_DEBUG_ONCE("ErrorStateKalmanFilter::measureMagneticField");

  // Compute innovation
  const double delta_yaw = wrapPi(yaw_meas - getYaw());

  // Choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
  constexpr double kEpsilon = 1e-6;
  const Quaterniond q = getQuaternion();

  bool can_use_A = false;
  const double SA0 = 2 * q.z();
  const double SA1 = 2 * q.y();
  const double SA2 = SA0 * q.w() + SA1 * q.x();
  const double SA3 = sqr(q.w()) + sqr(q.x()) - sqr(q.y()) - sqr(q.z());
  double SA4, SA5_inv;
  if (sqr(SA3) > kEpsilon)
  {
    SA4 = 1 / sqr(SA3);
    SA5_inv = sqr(SA2) * SA4 + 1;
    can_use_A = abs(SA5_inv) > kEpsilon;
  }

  bool can_use_B = false;
  const double SB0 = 2 * q.w();
  const double SB1 = 2 * q.x();
  const double SB2 = SB0 * q.z() + SB1 * q.y();
  const double SB4 = sqr(q.w()) + sqr(q.x()) - sqr(q.y()) - sqr(q.z());
  double SB3, SB5_inv;
  if (sqr(SB2) > kEpsilon)
  {
    SB3 = 1 / sqr(SB2);
    SB5_inv = SB3 * sqr(SB4) + 1;
    can_use_B = abs(SB5_inv) > kEpsilon;
  }

  // Compute output matrix
  RowVector4d H_yaw;
  if (can_use_A && (!can_use_B || abs(SA5_inv) >= abs(SB5_inv)))
  {
    const double SA5 = 1 / SA5_inv;
    const double SA6 = 1 / SA3;
    const double SA7 = SA2 * SA4;
    const double SA8 = 2 * SA7;
    const double SA9 = 2 * SA6;

    H_yaw(0) = SA5 * (SA0 * SA6 - SA8 * q.w());
    H_yaw(1) = SA5 * (SA1 * SA6 - SA8 * q.x());
    H_yaw(2) = SA5 * (SA1 * SA7 + SA9 * q.x());
    H_yaw(3) = SA5 * (SA0 * SA7 + SA9 * q.w());
  }
  else if (can_use_B && (!can_use_A || abs(SB5_inv) > abs(SA5_inv)))
  {
    const double SB5 = 1 / SB5_inv;
    const double SB6 = 1 / SB2;
    const double SB7 = SB3 * SB4;
    const double SB8 = 2 * SB7;
    const double SB9 = 2 * SB6;

    H_yaw(0) = -SB5 * (SB0 * SB6 - SB8 * q.z());
    H_yaw(1) = -SB5 * (SB1 * SB6 - SB8 * q.y());
    H_yaw(2) = -SB5 * (-SB1 * SB7 - SB9 * q.y());
    H_yaw(3) = -SB5 * (-SB0 * SB7 - SB9 * q.z());
  }
  else
  {
    throw runtime_error("Unable to compute output matrix.");
  }

  const auto Q_dtheta = getQ_dtheta();
  H_mag_.block<1, 3>(0, kDeltaThetaIdx) = H_yaw * Q_dtheta;

  // Update the quaternion states and covariance matrix
  return correct<1>(Scalard(delta_yaw), Scalard(yaw_var), H_mag_);
}

Matrix<double, 4, 3> ErrorStateKalmanFilter::getQ_dtheta() const
{
  const Vector4d qby2 = 0.5 * getHamilton();
  const double& w = qby2(0);
  const double& x = qby2(1);
  const double& y = qby2(2);
  const double& z = qby2(3);

  Matrix<double, 4, 3> Q_dtheta;
  Q_dtheta << -x, -y, -z, w, -z, y, z, w, -x, -y, x, w;

  return Q_dtheta;
}

Matrix<double, 3, 4> ErrorStateKalmanFilter::quatRotationDerivative(const Vector3d& a) const
{
  const Vector4d ham = getHamilton();
  const double& w = ham(0);
  const Vector3d v = ham.tail<3>();

  Matrix<double, 3, 4> res;
  res.block<3, 1>(0, 0) = 2 * (w * a - a.cross(v));
  res.block<3, 3>(0, 1) =
    2 * (a.dot(v) * I3 + v * a.transpose() - a * v.transpose() - w * et::crossMat(a));

  return res;
}

void ErrorStateKalmanFilter::injectErrorState(const DeltaStateVector& error_state)
{
  // (283) 観測した誤差をノミナル状態に反映
  const Vector3d dtheta = error_state.segment<3>(kDeltaThetaIdx);
  const Quaterniond q_dtheta = et::angleAxisToQuaternion(dtheta);
  x_.segment<3>(kPosIdx) += error_state.segment<3>(kDeltaPosIdx);
  x_.segment<3>(kVelIdx) += error_state.segment<3>(kDeltaVelIdx);
  x_.segment<4>(kQuatIdx) = et::quaternionToHamilton(getQuaternion() * q_dtheta).normalized();
  x_.segment<3>(kAccBiasIdx) += error_state.segment<3>(kDeltaAccBiasIdx);
  x_.segment<3>(kGyroBiasIdx) += error_state.segment<3>(kDeltaGyroBiasIdx);
  x_(kGravIdx) += error_state(kDeltaGravIdx);

  // (286) ESKFを初期化 (不要)
  // FIXME: これをやるとバグる問題．symmetriseを挟むと若干マシになるがそれでもやらないほうがマシ．
  // const Matrix3d G_theta = I3 - et::crossMat(0.5 * dtheta);
  // P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) =
  //   G_theta * P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) * G_theta.transpose();
  // et::symmetrise(P_);
}
}  // namespace state_estimation_eskf
