#include <iostream>

#include <tobas_math/core.hpp>
#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>

#include "../include/tobas_eskf/eskf.hpp"

#define E3 Diagonal3d(1, 1, 1)

using namespace std;
using namespace chrono;
using namespace Eigen;

namespace eskf
{
ErrorStateKalmanFilter::ErrorStateKalmanFilter() : x_history_(kStateHistoryTimeWindow)
{
  // 観測方程式の固定部分を埋める
  H_pos_.setZero();
  H_xy_.setZero();
  H_z_.setZero();
  H_vel_.setZero();
  H_pv_.setZero();
  H_theta_.setZero();
  H_mag_.setZero();
  H_grav_.setZero();

  H_pos_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_xy_.block<2, 2>(0, kDeltaPosIdx).diagonal().setOnes();
  H_z_(0, kDeltaAltIdx) = 1;
  H_vel_.block<3, 3>(0, kDeltaVelIdx).diagonal().setOnes();
  H_pv_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_pv_.block<3, 3>(3, kDeltaVelIdx).diagonal().setOnes();
  H_theta_.block<3, 3>(0, kDeltaThetaIdx).diagonal().setOnes();  // 回転の誤差を3Dベクトルとして観測
  H_mag_.block<3, 3>(0, kDeltaMagHardBiasIdx).diagonal().setOnes();
  H_grav_.block<3, 3>(0, kDeltaAccBiasIdx).diagonal().setOnes();
}

bool ErrorStateKalmanFilter::initialize(
  const Vector3d& init_pos,
  const Matrix3d& init_pos_cov,
  const Vector3d& init_vel,
  const Matrix3d& init_vel_cov,
  const Quaterniond& init_quat,
  const Matrix3d& init_dtheta_cov,
  const Vector3d& init_acc_bias,
  const Matrix3d& init_acc_bias_cov,
  const Vector3d& init_gyro_bias,
  const Matrix3d& init_gyro_bias_cov,
  const Vector3d& init_mag_hard_bias,
  const Matrix3d& init_mag_hard_bias_cov,
  const Matrix3d& init_mag_soft_bias,
  const Matrix6d& init_mag_soft_bias_cov,
  const double& init_grav,
  const double& init_grav_var,
  const steady_clock::time_point& time)
{
  // Set initial IMU time
  t_last_imu_ = time;

  // Fill the constant part of matrices
  P_.setZero();
  F_x_.setIdentity();
  G_.setIdentity();

  // Initialize states and covariances
  if (!initializePosition(init_pos, init_pos_cov))
    return false;
  if (!initializeVelocity(init_vel, init_vel_cov))
    return false;
  if (!initializeQuaternion(init_quat, init_dtheta_cov))
    return false;
  if (!initializeAccelBias(init_acc_bias, init_acc_bias_cov))
    return false;
  if (!initializeGyroBias(init_gyro_bias, init_gyro_bias_cov))
    return false;
  if (!initializeMagHardBias(init_mag_hard_bias, init_mag_hard_bias_cov))
    return false;
  if (!initializeMagSoftBias(init_mag_soft_bias, init_mag_soft_bias_cov))
    return false;
  if (!initializeGravity(init_grav, init_grav_var))
    return false;

  return true;
}

bool ErrorStateKalmanFilter::initializePosition(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov))
  {
    cerr << "Initial position covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kPosIdx) = value;
  P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx) = cov;
  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeVelocity(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov))
  {
    cerr << "Initial velocity covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kVelIdx) = value;
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) = cov;
  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeQuaternion(const Quaterniond& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov))
  {
    cerr << "Initial rotation covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<4>(kQuatIdx) = eigen::hamiltonFromQuaternion(value).normalized();
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = cov;
  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeAccelBias(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov))
  {
    cerr << "Initial accelerometer bias covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kAccBiasIdx) = value;
  P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx) = cov;
  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeGyroBias(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov))
  {
    cerr << "Initial gyroscope bias covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kGyroBiasIdx) = value;
  P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx) = cov;
  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeMagHardBias(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov))
  {
    cerr << "Initial magnetometer hard bias covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kMagHardBiasIdx) = value;
  P_.block<3, 3>(kDeltaMagHardBiasIdx, kDeltaMagHardBiasIdx) = cov;
  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeMagSoftBias(const Matrix3d& value, const Matrix6d& cov)
{
  if (!eigen::isSymmetricPositiveDefinite(value))
  {
    cerr << "Initial magnetometer soft bias matrix must be symmetric positive definite." << endl;
    return false;
  }

  if (!eigen::isSymmetricSemiPositiveDefinite(cov))
  {
    cerr << "Initial magnetometer soft bias covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<6>(kMagSoftBiasIdx) << value(0, 0), value(0, 1), value(0, 2), value(1, 1), value(1, 2), value(2, 2);
  P_.block<6, 6>(kDeltaMagSoftBiasIdx, kDeltaMagSoftBiasIdx) = cov;
  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeGravity(const double& value, const double& var)
{
  if (var < 0.)
  {
    cerr << "Initial gravity variance must be non-negative." << endl;
    return false;
  }

  x_(kGravIdx) = value;
  P_(kDeltaGravIdx, kDeltaGravIdx) = var;
  resetStateHistory();

  return true;
}

void ErrorStateKalmanFilter::enableJosephForm(bool enable)
{
  use_joseph_form_ = enable;
}

void ErrorStateKalmanFilter::enableCovInitialization(bool enable)
{
  do_cov_initialization_ = enable;
}

bool ErrorStateKalmanFilter::setAccBiasProcNoiseDensity(double value)
{
  if (value < 0.)
  {
    cerr << "The noise density of accelerometer bias process must be non-negative." << endl;
    return false;
  }

  acc_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setGyroBiasProcNoiseDensity(double value)
{
  if (value < 0.)
  {
    cerr << "The noise density of gyroscope bias process must be non-negative." << endl;
    return false;
  }

  gyro_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setMagHardBiasProcNoiseDensity(double value)
{
  if (value < 0.)
  {
    cerr << "The noise density of magnetometer hard-iron bias process must be non-negative." << endl;
    return false;
  }

  mag_hard_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setMagSoftBiasProcNoiseDensity(double value)
{
  if (value < 0.)
  {
    cerr << "The noise density of magnetometer soft-iron bias process must be non-negative." << endl;
    return false;
  }

  mag_soft_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setGravProcNoiseDensity(double value)
{
  if (value < 0.)
  {
    cerr << "The noise density of gravity process must be non-negative." << endl;
    return false;
  }

  grav_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setMagneticFieldRef(const Vector3d& mag_W)
{
  if (mag_W.norm() <= 0.)
  {
    cerr << "The norm of reference magnetic field must be positive." << endl;
    return false;
  }

  mag_W_ = mag_W.normalized();
  return true;
}

double ErrorStateKalmanFilter::measureIMU(
  const Vector3d& acc_meas,
  const Vector3d& gyro_meas,
  const Matrix3d& acc_cov,
  const Matrix3d& gyro_cov,
  const Matrix3d& grav_cov,
  const steady_clock::time_point& time)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(acc_cov));
  assert(eigen::isSymmetricSemiPositiveDefinite(gyro_cov));
  assert(eigen::isSymmetricPositiveDefinite(grav_cov));

  // サンプリングタイムを計算して時刻を更新
  const auto dt = duration<double>(time - t_last_imu_).count();  // [s]
  const auto dt2 = math::sqr(dt);
  t_last_imu_ = time;

  // クオータニオンの正規化のためにdt = 0を許容できない
  if (dt <= 0)
  {
    cerr << "IMU time gap must be positive: " << dt << " <= 0 [sec]" << endl;
    return INFINITY;
  }

  const Vector3d acc_B = acc_meas - getAccelBias(x_);
  const Vector3d gyro_B = gyro_meas - getGyroBias(x_);

  const Quaterniond delta_q = eigen::quaternionFromAngleAxis(gyro_B * dt);
  const Matrix3d delta_R = delta_q.toRotationMatrix();

  const Vector3d vel_W = getVelocity(x_);
  const Quaterniond q = getQuaternion(x_);
  const Matrix3d W_Rot_B = q.toRotationMatrix();
  const Vector3d acc_grav_W = W_Rot_B * acc_B + getGravVector(x_);

  // (260) ノミナル状態のキネマティクス
  x_.segment<3>(kPosIdx) += vel_W * dt;
  if (enable_second_integral_)
    x_.segment<3>(kPosIdx) += 0.5 * acc_grav_W * dt2;  // XXX: 積分誤差増大リスクあり
  x_.segment<3>(kVelIdx) += acc_grav_W * dt;
  x_.segment<4>(kQuatIdx) = eigen::hamiltonFromQuaternion(q * delta_q);

  // (270) ヤコビアンの可変部を更新
  F_x_.block<3, 3>(kDeltaPosIdx, kDeltaVelIdx).diagonal().fill(dt);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaThetaIdx) = -W_Rot_B * eigen::skew(acc_B * dt);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaAccBiasIdx) = -W_Rot_B * dt;
  F_x_(kDeltaVelIdx + 2, kDeltaGravIdx) = -dt;
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = delta_R.transpose();
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaGyroBiasIdx).diagonal().fill(-dt);

  // (269)第一項: 共分散行列の予測値を更新
  P_ = F_x_ * P_ * F_x_.transpose();  // TODO: 必要な部分のみ計算

  // (269)第二項: プロセスノイズを印加
  // TODO: 異なるdtに対応させるため，プロセスノイズの分散をノイズ密度で定義
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) += W_Rot_B * acc_cov * W_Rot_B.transpose() * dt2;
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) += W_Rot_B * gyro_cov * W_Rot_B.transpose() * dt2;
  P_.diagonal().segment<3>(kDeltaAccBiasIdx).array() += math::sqr(acc_bias_proc_noise_density_) * dt;
  P_.diagonal().segment<3>(kDeltaGyroBiasIdx).array() += math::sqr(gyro_bias_proc_noise_density_) * dt;
  P_.diagonal().segment<3>(kDeltaMagHardBiasIdx).array() += math::sqr(mag_hard_bias_proc_noise_density_) * dt;
  P_.diagonal().segment<6>(kDeltaMagSoftBiasIdx).array() += math::sqr(mag_soft_bias_proc_noise_density_) * dt;
  P_(kDeltaGravIdx, kDeltaGravIdx) += math::sqr(grav_proc_noise_density_) * dt;

  // Apply constraints to avoid numerical errors
  applyConstraints();

  // 状態の履歴を保存
  x_history_.add(time, x_);

  // 重力方向の観測: 加速度と姿勢には等式関係 (= 出力方程式) があるため，カルマンフィルタ理論に則って補正を行う．
  return measureGravity(acc_meas, grav_cov, time);
}

double ErrorStateKalmanFilter::measurePosition(
  const Vector3d& pos_meas,
  const Matrix3d& pos_cov,
  const Vector3d& offset,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Vector3d delta_pos = pos_meas - getPosition(x, offset);

  // 姿勢による偏微分
  const auto dqvq_dq = quatRotationDerivative(x, offset);
  const auto Q_dtheta = getQ_dtheta(x);
  H_pos_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  return correct(delta_pos, pos_cov, H_pos_);
}

double
ErrorStateKalmanFilter::measureXY(const Vector2d& xy_meas, const Matrix2d& xy_cov, const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Vector2d delta_xy = xy_meas - getXY(x);
  return correct(delta_xy, xy_cov, H_xy_);
}

double
ErrorStateKalmanFilter::measureAltitude(const double& z_meas, const double& z_var, const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const double delta_z = z_meas - getAltitude(x);
  return correct(Scalard(delta_z), Scalard(z_var), H_z_);
}

double ErrorStateKalmanFilter::measureVelocity(
  const Vector3d& vel_meas,
  const Matrix3d& vel_cov,
  const Vector3d& offset,
  const Vector3d& gyro_meas,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Vector3d gyro_nominal = gyro_meas - getGyroBias(x);
  const Vector3d gyro_offset = gyro_nominal.cross(offset);
  const Vector3d vel_nominal = getVelocity(x) + getQuaternion(x) * gyro_offset;
  const Vector3d delta_vel = vel_meas - vel_nominal;

  // 姿勢による偏微分
  const auto dqvq_dq = quatRotationDerivative(x, gyro_offset);
  const auto Q_dtheta = getQ_dtheta(x);
  H_vel_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  // ジャイロバイアスによる偏微分
  H_vel_.block<3, 3>(0, kDeltaGyroBiasIdx) = getDCM(x) * eigen::skew(offset);

  return correct(delta_vel, vel_cov, H_vel_);
}

double ErrorStateKalmanFilter::measurePosVel(
  const Vector3d& pos_meas,
  const Matrix3d& pos_cov,
  const Vector3d& vel_meas,
  const Matrix3d& vel_cov,
  const Vector3d& offset,
  const Vector3d& gyro_meas,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  // 観測誤差
  Vector6d delta;
  const Vector3d gyro_nominal = gyro_meas - getGyroBias(x);
  const Vector3d gyro_offset = gyro_nominal.cross(offset);
  const Vector3d vel_nominal = getVelocity(x) + getQuaternion(x) * gyro_offset;
  delta.head<3>() = pos_meas - getPosition(x, offset);  // 位置の誤差
  delta.tail<3>() = vel_meas - vel_nominal;             // 速度の誤差

  // 観測方程式
  const auto Q_dtheta = getQ_dtheta(x);
  const auto pos_q_deriv = quatRotationDerivative(x, offset);
  const auto vel_q_deriv = quatRotationDerivative(x, gyro_offset);
  H_pv_.block<3, 3>(0, kDeltaThetaIdx) = pos_q_deriv * Q_dtheta;  // 位置の姿勢による偏微分
  H_pv_.block<3, 3>(3, kDeltaThetaIdx) = vel_q_deriv * Q_dtheta;  // 速度の姿勢による偏微分
  H_pv_.block<3, 3>(0, kDeltaGyroBiasIdx) = getDCM(x) * eigen::skew(offset);

  // 共分散
  Matrix6d cov;
  cov.topLeftCorner<3, 3>() = pos_cov;
  cov.bottomRightCorner<3, 3>() = vel_cov;
  cov.topRightCorner<3, 3>().setZero();
  cov.bottomLeftCorner<3, 3>().setZero();

  // 事後推定を更新
  return correct(delta, cov, H_pv_);
}

double ErrorStateKalmanFilter::measureQuaternion(
  const Quaterniond& q_meas,
  const Matrix3d& theta_cov,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Quaterniond q_nominal = getQuaternion(x);
  const Quaterniond q_error = q_nominal.conjugate() * q_meas;  // 回転の誤差
  const Vector3d delta_theta = eigen::angleAxisFromQuaternion(q_error);

  return correct(delta_theta, theta_cov, H_theta_);
}

double ErrorStateKalmanFilter::measureMagneticField(
  const Vector3d& mag_meas,
  const Matrix3d& mag_cov,
  const steady_clock::time_point& time)
{
  if (mag_W_.norm() == 0.)
  {
    cerr << "Reference magnetic field is not set." << endl;
    return INFINITY;
  }

  const auto& x = x_history_.closestAfterValue(time);

  const Vector3d mag_B = getQuaternion(x).inverse() * mag_W_;
  const Vector3d m_b = getMagHardBias(x);
  const Matrix3d T = getMagSoftBias(x);

  const auto& mx = mag_B.x();
  const auto& my = mag_B.y();
  const auto& mz = mag_B.z();

  // 観測誤差
  const auto mag_pred = T * mag_B + m_b;
  const Vector3d delta_mag = mag_meas - mag_pred;

  // 観測方程式
  H_mag_.block<3, 3>(0, kDeltaThetaIdx) = T * eigen::skew(2 * mag_B);
  H_mag_.block<3, 6>(0, kDeltaMagSoftBiasIdx) << mx, my, mz, 0, 0, 0, 0, mx, 0, my, mz, 0, 0, 0, mx, 0, my, mz;

  // 事後推定を更新
  return correct(delta_mag, mag_cov, H_mag_);
}

Matrix<double, 4, 3> ErrorStateKalmanFilter::getQ_dtheta(const StateVector& x) const
{
  const Vector4d qby2 = 0.5 * getHamilton(x);
  const auto& qw = qby2(0);
  const auto& qx = qby2(1);
  const auto& qy = qby2(2);
  const auto& qz = qby2(3);

  Matrix<double, 4, 3> Q_dtheta;
  Q_dtheta << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw;

  return Q_dtheta;
}

Matrix<double, 3, 4> ErrorStateKalmanFilter::quatRotationDerivative(const StateVector& x, const Vector3d& a) const
{
  const Vector4d ham = getHamilton(x);
  const double& w = ham(0);
  const Vector3d v = ham.tail<3>();

  Matrix<double, 3, 4> res;
  res.block<3, 1>(0, 0) = 2 * (w * a - a.cross(v));
  res.block<3, 3>(0, 1) = 2 * (a.dot(v) * E3 + v * a.transpose() - a * v.transpose() - w * eigen::skew(a));

  return res;
}

RowVector4d ErrorStateKalmanFilter::hamiltonToYawOutputMatrix(const StateVector& x) const
{
  // Choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw
  constexpr double kEpsilon = 1e-6;
  const Quaterniond q = getQuaternion(x);

  bool can_use_A = false;
  const auto SA0 = 2 * q.z();
  const auto SA1 = 2 * q.y();
  const auto SA2 = SA0 * q.w() + SA1 * q.x();
  const auto SA3 = math::sqr(q.w()) + math::sqr(q.x()) - math::sqr(q.y()) - math::sqr(q.z());
  double SA4, SA5_inv;
  if (math::sqr(SA3) > kEpsilon)
  {
    SA4 = 1 / math::sqr(SA3);
    SA5_inv = math::sqr(SA2) * SA4 + 1;
    can_use_A = fabs(SA5_inv) > kEpsilon;
  }

  bool can_use_B = false;
  const auto SB0 = 2 * q.w();
  const auto SB1 = 2 * q.x();
  const auto SB2 = SB0 * q.z() + SB1 * q.y();
  const auto SB4 = math::sqr(q.w()) + math::sqr(q.x()) - math::sqr(q.y()) - math::sqr(q.z());
  double SB3, SB5_inv;
  if (math::sqr(SB2) > kEpsilon)
  {
    SB3 = 1 / math::sqr(SB2);
    SB5_inv = SB3 * math::sqr(SB4) + 1;
    can_use_B = fabs(SB5_inv) > kEpsilon;
  }

  // Compute output matrix
  RowVector4d H;
  if (can_use_A && (!can_use_B || fabs(SA5_inv) >= fabs(SB5_inv)))
  {
    const auto SA5 = 1 / SA5_inv;
    const auto SA6 = 1 / SA3;
    const auto SA7 = SA2 * SA4;
    const auto SA8 = 2 * SA7;
    const auto SA9 = 2 * SA6;

    H(0) = SA5 * (SA0 * SA6 - SA8 * q.w());
    H(1) = SA5 * (SA1 * SA6 - SA8 * q.x());
    H(2) = SA5 * (SA1 * SA7 + SA9 * q.x());
    H(3) = SA5 * (SA0 * SA7 + SA9 * q.w());
  }
  else if (can_use_B && (!can_use_A || fabs(SB5_inv) > fabs(SA5_inv)))
  {
    const auto SB5 = 1 / SB5_inv;
    const auto SB6 = 1 / SB2;
    const auto SB7 = SB3 * SB4;
    const auto SB8 = 2 * SB7;
    const auto SB9 = 2 * SB6;

    H(0) = -SB5 * (SB0 * SB6 - SB8 * q.z());
    H(1) = -SB5 * (SB1 * SB6 - SB8 * q.y());
    H(2) = -SB5 * (-SB1 * SB7 - SB9 * q.y());
    H(3) = -SB5 * (-SB0 * SB7 - SB9 * q.z());
  }
  else
  {
    cerr << "Unable to compute the output matrix of yaw angle observation." << endl;
    return RowVector4d::Zero();
  }

  return H;
}

void ErrorStateKalmanFilter::applyConstraints()
{
  // 状態の等式制約
  applyStateEqualityConstraints();

  // 状態の不等式制約
  applyStateInequalityConstraints();

  // 共分散行列は対称行列でなければならない
  eigen::symmetrise(P_);
}

void ErrorStateKalmanFilter::applyStateEqualityConstraints()
{
  // クオータニオンのノルムは1
  x_.segment<4>(kQuatIdx) = x_.segment<4>(kQuatIdx).normalized();
}

void ErrorStateKalmanFilter::applyStateInequalityConstraints()
{
  // 事前知識を用いて状態が最低限ありえない値にはならないようにする
  x_.segment<3>(kAccBiasIdx) = x_.segment<3>(kAccBiasIdx).cwiseMax(-kMaxAccBias).cwiseMin(kMaxAccBias);
  x_.segment<3>(kGyroBiasIdx) = x_.segment<3>(kGyroBiasIdx).cwiseMax(-kMaxGyroBias).cwiseMin(kMaxGyroBias);
  x_.segment<3>(kMagHardBiasIdx) = x_.segment<3>(kMagHardBiasIdx).cwiseMax(-kMaxMagHardBias).cwiseMin(kMaxMagHardBias);
  x_(kGravIdx) = clamp(x_(kGravIdx), kMinGravity, kMaxGravity);
}

void ErrorStateKalmanFilter::resetStateHistory()
{
  x_history_.clear();
  x_history_.add(t_last_imu_, x_);
}

double ErrorStateKalmanFilter::measureGravity(
  const Vector3d& acc_meas,
  const Matrix3d& grav_cov,
  const steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Matrix3d R_B_W = getDCM(x).transpose();
  const Vector3d grav_B = R_B_W * getGravVector(x);
  const Vector3d acc_ref = getAccelBias(x) - grav_B;  // 動的な加速度なしで観測されるべき加速度
  const Vector3d delta_acc = acc_meas - acc_ref;  // TODO: モデルから推定した動的加速度を引いた値を観測値とする

  H_grav_.block<3, 3>(0, kDeltaThetaIdx) = -eigen::skew(2 * grav_B);
  H_grav_.col(kDeltaGravIdx) = R_B_W.col(2);
  return correct(delta_acc, grav_cov, H_grav_);
}
}  // namespace eskf
