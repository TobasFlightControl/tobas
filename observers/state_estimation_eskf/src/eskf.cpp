#include <dh_std_tools/math.hpp>
#include <dh_eigen_tools/linalg.hpp>
#include <dh_eigen_tools/geometry.hpp>

#include "../include/state_estimation_eskf/eskf.hpp"

#define I3 Matrix3d::Identity()

using namespace std;
using namespace Eigen;
using namespace dh_std;

namespace et = eigen_tools;

namespace state_estimation_eskf
{
ErrorStateKalmanFilter::ErrorStateKalmanFilter()
{
  // 観測方程式の固定部分を埋める
  H_pos_.setZero();
  H_xy_.setZero();
  H_z_.setZero();
  H_vel_.setZero();
  H_theta_.setZero();
  H_acc_.setZero();
  H_mag_rpy_.setZero();
  H_mag_yaw_.setZero();

  H_pos_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_xy_.block<2, 2>(0, kDeltaPosIdx).diagonal().setOnes();
  H_z_(0, kDeltaAltIdx) = 1.;
  H_vel_.block<3, 3>(0, kDeltaVelIdx).diagonal().setOnes();
  H_theta_.block<3, 3>(0, kDeltaThetaIdx).diagonal().setOnes();  // 回転の誤差を3Dベクトルとして観測
}

void ErrorStateKalmanFilter::initialize(
  const Vector3d& grav_W,
  const Vector3d& mag_W,
  const Vector3d& init_pos,
  const Vector3d& init_vel,
  const Quaterniond& init_quat,
  const Matrix3d& init_pos_cov,
  const Matrix3d& init_vel_cov,
  const Matrix3d& init_dtheta_cov,
  const Matrix3d& init_acc_bias_cov,
  const Matrix3d& init_gyro_bias_cov)
{
  assert(grav_W.z() < 0.);
  assert(et::isSymmetricSemiPositiveDefinite(init_pos_cov));
  assert(et::isSymmetricSemiPositiveDefinite(init_vel_cov));
  assert(et::isSymmetricSemiPositiveDefinite(init_dtheta_cov));
  assert(et::isSymmetricSemiPositiveDefinite(init_acc_bias_cov));
  assert(et::isSymmetricSemiPositiveDefinite(init_gyro_bias_cov));

  grav_W_ = grav_W;
  mag_W_ = mag_W;

  // ノミナル状態を初期化
  nominal_state_.setZero();
  nominal_state_.block<3, 1>(kPosIdx, 0) = init_pos;
  nominal_state_.block<3, 1>(kVelIdx, 0) = init_vel;
  nominal_state_.block<4, 1>(kQuatIdx, 0) = et::quaternionToHamilton(init_quat).normalized();

  // 共分散行列を初期化
  P_.setZero();
  P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx) = init_pos_cov;
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) = init_vel_cov;
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = init_dtheta_cov;
  P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx) = init_acc_bias_cov;
  P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx) = init_gyro_bias_cov;

  // (270) ヤコビアンの不変部分を埋める
  F_x_.setZero();
  F_x_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx).diagonal().setOnes();
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx).diagonal().setOnes();
  F_x_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx).diagonal().setOnes();
  F_x_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx).diagonal().setOnes();
}

Vector3d ErrorStateKalmanFilter::getXYZ() const
{
  return nominal_state_.block<3, 1>(kPosIdx, 0);
}

Vector2d ErrorStateKalmanFilter::getXY() const
{
  return nominal_state_.block<2, 1>(kPosIdx, 0);
}

double ErrorStateKalmanFilter::getAltitude() const
{
  return nominal_state_(kAltIdx);
}

Vector3d ErrorStateKalmanFilter::getVelocity() const
{
  return nominal_state_.block<3, 1>(kVelIdx, 0);
}

Quaterniond ErrorStateKalmanFilter::getQuaternion() const
{
  return et::hamiltonToQuaternion(getHamilton());
}

Vector3d ErrorStateKalmanFilter::getAccelBias() const
{
  return nominal_state_.block<3, 1>(kAccBiasIdx, 0);
}

Vector3d ErrorStateKalmanFilter::getGyroBias() const
{
  return nominal_state_.block<3, 1>(kGyroBiasIdx, 0);
}

Matrix3d ErrorStateKalmanFilter::getDCM() const
{
  return getQuaternion().toRotationMatrix();
}

double ErrorStateKalmanFilter::getYaw() const
{
  const auto R_W_B = getDCM();
  return atan2(R_W_B(1, 0), R_W_B(0, 0));
}

Matrix3d ErrorStateKalmanFilter::getPositionCovariance() const
{
  return P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx);
}

Matrix3d ErrorStateKalmanFilter::getVelocityCovariance() const
{
  return P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx);
}

Matrix3d ErrorStateKalmanFilter::getOrientationCovariance() const
{
  return P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx);
}

Matrix3d ErrorStateKalmanFilter::getAccelBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx);
}

Matrix3d ErrorStateKalmanFilter::getGyroBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx);
}

void ErrorStateKalmanFilter::predictIMU(
  const Vector3d& acc_meas,
  const Vector3d& gyro_meas,
  const double& acc_noise_var,
  const double& gyro_noise_var,
  const double& acc_bias_noise_var,
  const double& gyro_bias_noise_var,
  const double& dt)
{
  assert(acc_noise_var > 0.);
  assert(gyro_noise_var > 0.);
  assert(acc_bias_noise_var > 0.);
  assert(gyro_bias_noise_var > 0.);
  assert(dt > 0.);  // クオータニオンの正規化のためにdt = 0を許容できない

  const Matrix3d Rot = getDCM();
  const Vector3d acc_B = acc_meas - getAccelBias();
  const Vector3d acc_W = Rot * acc_B;
  const Vector3d delta_theta = (gyro_meas - getGyroBias()) * dt;
  const Quaterniond q_delta_theta = et::angleAxisToQuaternion(delta_theta);
  const Matrix3d R_delta_theta = q_delta_theta.toRotationMatrix();

  // (260) ノミナル状態のキネマティクス
  nominal_state_.block<3, 1>(kPosIdx, 0) += getVelocity() * dt + 0.5 * (acc_W + grav_W_) * sqr(dt);
  nominal_state_.block<3, 1>(kVelIdx, 0) += (acc_W + grav_W_) * dt;
  nominal_state_.block<4, 1>(kQuatIdx, 0) =
    et::quaternionToHamilton(getQuaternion() * q_delta_theta).normalized();

  // (270) ヤコビアンの可変部を更新
  F_x_.block<3, 3>(kDeltaPosIdx, kDeltaVelIdx).diagonal().fill(dt);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaThetaIdx) = -Rot * et::crossMat(acc_B) * dt;
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaAccBiasIdx) = -Rot * dt;
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = R_delta_theta.transpose();
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaGyroBiasIdx).diagonal().fill(-dt);

  // (269)第一項: 共分散行列の予測値を更新
  // TODO: sympyを用いて行列積を効率化
  P_ = F_x_ * P_ * F_x_.transpose();

  // 無理やり対称化 (これが必須)
  // プロセスノイズを加える前に対称化する必要がある？
  et::symmetrise(P_);

  // (269)第二項: プロセスノイズを印加
  P_.diagonal().block<3, 1>(kDeltaVelIdx, 0).array() += acc_noise_var * sqr(dt);
  P_.diagonal().block<3, 1>(kDeltaThetaIdx, 0).array() += gyro_noise_var * sqr(dt);
  P_.diagonal().block<3, 1>(kDeltaAccBiasIdx, 0).array() += acc_bias_noise_var;
  P_.diagonal().block<3, 1>(kDeltaGyroBiasIdx, 0).array() += gyro_bias_noise_var;

  // NaN検出
  assert(et::isFinite(nominal_state_));
  assert(et::isFinite(F_x_));
  assert(et::isFinite(P_));
}

void ErrorStateKalmanFilter::measurePosition(
  const Vector3d& pos_meas,
  const Matrix3d& pos_cov,
  const Vector3d& offset)
{
  const Vector3d pos_nominal = getXYZ() + getQuaternion() * offset;
  const Vector3d delta_pos = pos_meas - pos_nominal;

  // 姿勢による偏微分
  const auto dqvq_dq = quatRotationDerivative(offset);
  const auto Q_dtheta = getQ_dtheta();
  H_pos_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  correct<3>(delta_pos, pos_cov, H_pos_);
}

void ErrorStateKalmanFilter::measureXY(const Vector2d& xy_meas, const Matrix2d& xy_cov)
{
  const Vector2d delta_xy = xy_meas - getXY();
  correct<2>(delta_xy, xy_cov, H_xy_);
}

void ErrorStateKalmanFilter::measureAltitude(const double& z_meas, const double& z_var)
{
  const double delta_z = z_meas - getAltitude();
  correct<1>(Scalar(delta_z), Scalar(z_var), H_z_);
}

void ErrorStateKalmanFilter::measureVelocity(const Vector3d& vel_meas, const Matrix3d& vel_cov)
{
  measureVelocity(vel_meas, vel_cov, Vector3d::Zero(), Vector3d::Zero());
}

void ErrorStateKalmanFilter::measureVelocity(
  const Vector3d& vel_meas,
  const Matrix3d& vel_cov,
  const Vector3d& gyro_meas,
  const Vector3d& offset)
{
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

  correct<3>(delta_vel, vel_cov, H_vel_);
}

void ErrorStateKalmanFilter::measureQuaternion(const Quaterniond& q_meas, const Matrix3d& theta_cov)
{
  const Quaterniond q_nominal = getQuaternion();
  const Quaterniond q_error = q_nominal.conjugate() * q_meas;  // 回転の誤差
  const Vector3d delta_theta = et::quaternionToAngleAxis(q_error);

  correct<3>(delta_theta, theta_cov, H_theta_);
}

void ErrorStateKalmanFilter::measureAcceleration(const Vector3d& acc_meas, const Matrix3d& acc_cov)
{
  const Quaterniond Q_W_B = getQuaternion();
  const Vector3d grav_B = Q_W_B.conjugate() * grav_W_;
  const Vector3d acc_nominal = -grav_B;
  const Vector3d delta_acc = acc_meas - acc_nominal;

  H_acc_.block(0, kDeltaThetaIdx, 3, 3) = -2 * et::crossMat(grav_B);
  correct<3>(delta_acc, acc_cov, H_acc_);
}

void ErrorStateKalmanFilter::measureMagneticField(const Vector3d& mag_meas, const Matrix3d& mag_cov)
{
  const Quaterniond Q_W_B = getQuaternion();
  const Vector3d mag_B = Q_W_B.conjugate() * mag_W_;
  const Vector3d delta_mag = mag_meas - mag_B;

  H_mag_rpy_.block<3, 3>(0, kDeltaThetaIdx) = 2 * et::crossMat(mag_B);
  correct<3>(delta_mag, mag_cov, H_mag_rpy_);
}

void ErrorStateKalmanFilter::measureMagneticField(
  const double& mag_meas_x,
  const double& mag_meas_y,
  const double& yaw_var)
{
  assert(yaw_var > 0.);

  // Compute innovation
  const double yaw_meas = wrapPi(atan2(mag_W_.y(), mag_W_.x()) - atan2(mag_meas_y, mag_meas_x));
  const double delta_yaw = wrapPi(yaw_meas - getYaw());

  // Choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
  constexpr double epsilon = 1e-6;
  const Quaterniond q = getQuaternion();

  bool can_use_A = false;
  const double SA0 = 2 * q.z();
  const double SA1 = 2 * q.y();
  const double SA2 = SA0 * q.w() + SA1 * q.x();
  const double SA3 = sqr(q.w()) + sqr(q.x()) - sqr(q.y()) - sqr(q.z());
  double SA4, SA5_inv;
  if (sqr(SA3) > epsilon)
  {
    SA4 = 1 / sqr(SA3);
    SA5_inv = sqr(SA2) * SA4 + 1;
    can_use_A = abs(SA5_inv) > epsilon;
  }

  bool can_use_B = false;
  const double SB0 = 2 * q.w();
  const double SB1 = 2 * q.x();
  const double SB2 = SB0 * q.z() + SB1 * q.y();
  const double SB4 = sqr(q.w()) + sqr(q.x()) - sqr(q.y()) - sqr(q.z());
  double SB3, SB5_inv;
  if (sqr(SB2) > epsilon)
  {
    SB3 = 1 / sqr(SB2);
    SB5_inv = SB3 * sqr(SB4) + 1;
    can_use_B = abs(SB5_inv) > epsilon;
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
    return;
  }

  const auto Q_dtheta = getQ_dtheta();
  H_mag_yaw_.block<1, 3>(0, kDeltaThetaIdx) = H_yaw * Q_dtheta;

  // Update the quaternion states and covariance matrix
  correct<1>(Scalar(delta_yaw), Scalar(yaw_var), H_mag_yaw_);
}

Vector4d ErrorStateKalmanFilter::getHamilton() const
{
  return nominal_state_.block<4, 1>(kQuatIdx, 0);
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
  const Vector3d v = ham.block<3, 1>(1, 0);

  Matrix<double, 3, 4> res;
  res.block<3, 1>(0, 0) = 2 * (w * a - a.cross(v));
  res.block<3, 3>(0, 1) =
    2 * (a.dot(v) * I3 + v * a.transpose() - a * v.transpose() - w * et::crossMat(a));

  return res;
}

void ErrorStateKalmanFilter::injectErrorState(const DeltaStateVector& error_state)
{
  // (283) 観測した誤差をノミナル状態に反映
  const Vector3d dtheta = error_state.block<3, 1>(kDeltaThetaIdx, 0);
  const Quaterniond q_dtheta = et::angleAxisToQuaternion(dtheta);
  nominal_state_.block<3, 1>(kPosIdx, 0) += error_state.block<3, 1>(kDeltaPosIdx, 0);
  nominal_state_.block<3, 1>(kVelIdx, 0) += error_state.block<3, 1>(kDeltaVelIdx, 0);
  nominal_state_.block<4, 1>(kQuatIdx, 0) =
    et::quaternionToHamilton(getQuaternion() * q_dtheta).normalized();
  nominal_state_.block<3, 1>(kAccBiasIdx, 0) += error_state.block<3, 1>(kDeltaAccBiasIdx, 0);
  nominal_state_.block<3, 1>(kGyroBiasIdx, 0) += error_state.block<3, 1>(kDeltaGyroBiasIdx, 0);

  // (286) ESKFを初期化 (不要)
  // FIXME: これをやるとバグる問題．symmetriseを挟むと若干マシになるがそれでもやらないほうがマシ．
  // const Matrix3d G_theta = I3 - et::crossMat(0.5 * dtheta);
  // P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) =
  //   G_theta * P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) * G_theta.transpose();
  // et::symmetrise(P_);
}
}  // namespace state_estimation_eskf
