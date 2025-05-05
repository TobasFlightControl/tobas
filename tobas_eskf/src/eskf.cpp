#include "../include/tobas_eskf/eskf.hpp"

#include <iostream>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_math/core.hpp>

#define E3 Diagonal3d(1, 1, 1)

using namespace std;
using namespace chrono;
using namespace Eigen;

namespace eskf
{
ErrorStateKalmanFilter::ErrorStateKalmanFilter() : x_history_(kStateHistoryTimeWindow), stopwatch_(100)
{
  // 観測方程式の固定部分を埋める
  H_pos_.setZero();
  H_xy_.setZero();
  H_z_.setZero();
  H_vel_.setZero();
  H_pv_.setZero();
  H_theta_.setZero();
  H_mag_.setZero();
  H_yaw_.setZero();
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
  if (!initializePosition(init_pos, init_pos_cov)) {
    return false;
  }
  if (!initializeVelocity(init_vel, init_vel_cov)) {
    return false;
  }
  if (!initializeQuaternion(init_quat, init_dtheta_cov)) {
    return false;
  }
  if (!initializeAccelBias(init_acc_bias, init_acc_bias_cov)) {
    return false;
  }
  if (!initializeGyroBias(init_gyro_bias, init_gyro_bias_cov)) {
    return false;
  }
  if (!initializeMagHardBias(init_mag_hard_bias, init_mag_hard_bias_cov)) {
    return false;
  }
  if (!initializeMagSoftBias(init_mag_soft_bias, init_mag_soft_bias_cov)) {
    return false;
  }
  if (!initializeGravity(init_grav, init_grav_var)) {
    return false;
  }

  return true;
}

bool ErrorStateKalmanFilter::initializePosition(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov)) {
    cerr << "Initial position covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kPosIdx) = value;

  P_.middleRows<3>(kDeltaPosIdx).setZero();
  P_.middleCols<3>(kDeltaPosIdx).setZero();
  P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx) = cov;

  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeVelocity(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov)) {
    cerr << "Initial velocity covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kVelIdx) = value;

  P_.middleRows<3>(kDeltaVelIdx).setZero();
  P_.middleCols<3>(kDeltaVelIdx).setZero();
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) = cov;

  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeQuaternion(const Quaterniond& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov)) {
    cerr << "Initial rotation covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<4>(kQuatIdx) = eigen::hamiltonFromQuaternion(value).normalized();

  P_.middleRows<3>(kDeltaThetaIdx).setZero();
  P_.middleCols<3>(kDeltaThetaIdx).setZero();
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = cov;

  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeAccelBias(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov)) {
    cerr << "Initial accelerometer bias covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kAccBiasIdx) = value;

  P_.middleRows<3>(kDeltaAccBiasIdx).setZero();
  P_.middleCols<3>(kDeltaAccBiasIdx).setZero();
  P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx) = cov;

  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeGyroBias(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov)) {
    cerr << "Initial gyroscope bias covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kGyroBiasIdx) = value;

  P_.middleRows<3>(kDeltaGyroBiasIdx).setZero();
  P_.middleCols<3>(kDeltaGyroBiasIdx).setZero();
  P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx) = cov;

  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeMagHardBias(const Vector3d& value, const Matrix3d& cov)
{
  if (!eigen::isSymmetricSemiPositiveDefinite(cov)) {
    cerr << "Initial magnetometer hard bias covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  x_.segment<3>(kMagHardBiasIdx) = value;

  P_.middleRows<3>(kDeltaMagHardBiasIdx).setZero();
  P_.middleCols<3>(kDeltaMagHardBiasIdx).setZero();
  P_.block<3, 3>(kDeltaMagHardBiasIdx, kDeltaMagHardBiasIdx) = cov;

  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeMagSoftBias(const Matrix3d& value, const Matrix6d& cov)
{
  if (!eigen::isSymmetricPositiveDefinite(value)) {
    cerr << "Initial magnetometer soft bias matrix must be symmetric positive definite." << endl;
    return false;
  }

  if (!eigen::isSymmetricSemiPositiveDefinite(cov)) {
    cerr << "Initial magnetometer soft bias covariance must be symmetric semi-positive definite." << endl;
    return false;
  }

  setMagSoftBiasFromMatrix(value);

  P_.middleRows<6>(kDeltaMagSoftBiasIdx).setZero();
  P_.middleCols<6>(kDeltaMagSoftBiasIdx).setZero();
  P_.block<6, 6>(kDeltaMagSoftBiasIdx, kDeltaMagSoftBiasIdx) = cov;

  resetStateHistory();

  return true;
}

bool ErrorStateKalmanFilter::initializeGravity(const double& value, const double& var)
{
  if (var < 0.) {
    cerr << "Initial gravity variance must be non-negative." << endl;
    return false;
  }

  x_(kGravIdx) = value;

  P_.row(kDeltaGravIdx).setZero();
  P_.col(kDeltaGravIdx).setZero();
  P_(kDeltaGravIdx, kDeltaGravIdx) = var;

  resetStateHistory();

  return true;
}

void ErrorStateKalmanFilter::enableSecondIntegral(bool enable)
{
  enable_second_integral_ = enable;
}

void ErrorStateKalmanFilter::enableCovSymmetrisation(bool enable)
{
  enable_cov_symmetrisation_ = enable;
}

void ErrorStateKalmanFilter::enableCovInitialization(bool enable)
{
  enable_cov_initialization_ = enable;
}

void ErrorStateKalmanFilter::enableJosephForm(bool enable)
{
  enable_joseph_form_ = enable;
}

bool ErrorStateKalmanFilter::setAccBiasProcNoiseDensity(double value)
{
  if (value < 0.) {
    cerr << "The noise density of accelerometer bias process must be non-negative." << endl;
    return false;
  }

  acc_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setGyroBiasProcNoiseDensity(double value)
{
  if (value < 0.) {
    cerr << "The noise density of gyroscope bias process must be non-negative." << endl;
    return false;
  }

  gyro_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setMagHardBiasProcNoiseDensity(double value)
{
  if (value < 0.) {
    cerr << "The noise density of magnetometer hard-iron bias process must be non-negative." << endl;
    return false;
  }

  mag_hard_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setMagSoftBiasProcNoiseDensity(double value)
{
  if (value < 0.) {
    cerr << "The noise density of magnetometer soft-iron bias process must be non-negative." << endl;
    return false;
  }

  mag_soft_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setGravProcNoiseDensity(double value)
{
  if (value < 0.) {
    cerr << "The noise density of gravity process must be non-negative." << endl;
    return false;
  }

  grav_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setMagneticFieldRef(const Vector3d& mag_W)
{
  if (mag_W.norm() <= 0.) {
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
  if (dt <= 0) {
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

  // IMUの共分散を補正
  const auto acc_cov_fixed = eigen::nearestPositiveDefinite(acc_cov, math::sqr(kMinAccStddev));
  const auto gyro_cov_fixed = eigen::nearestPositiveDefinite(gyro_cov, math::sqr(kMinGyroStddev));

  // (260) ノミナル状態のキネマティクス
  x_.segment<3>(kPosIdx) += vel_W * dt;
  if (enable_second_integral_) {
    x_.segment<3>(kPosIdx) += 0.5 * acc_grav_W * dt2;  // XXX: 積分誤差増大リスクあり
  }
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
  // stopwatch_.start();
  P_ = F_x_ * P_.selfadjointView<Lower>() * F_x_.transpose();  // TODO: 必要な部分のみ計算
  // stopwatch_.stop();

  // (269)第二項: プロセスノイズを印加
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) += W_Rot_B * acc_cov_fixed * W_Rot_B.transpose() * dt2;
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) += W_Rot_B * gyro_cov_fixed * W_Rot_B.transpose() * dt2;
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
  // 自由落下中もしくは加速度が大きすぎる場合は全く姿勢を反映していない恐れがあるため，重力方向の観測を行うのはその間の加速度に限る．
  const auto acc_norm = acc_meas.norm();
  const auto gravity = getGravity(x_);
  if (acc_norm < kDoMeasGravMinGValue * gravity) {
    cerr << "Attitude correction cannot be performed because accel norm is lower than " << kDoMeasGravMinGValue << "G. "
         << endl;
    return INFINITY;
  }
  else if (acc_norm > kDoMeasGravMaxGValue * gravity) {
    cerr << "Attitude correction cannot be performed because accel norm is greater than " << kDoMeasGravMaxGValue
         << "G. " << endl;
    return INFINITY;
  }
  else {
    return measureGravity(acc_meas, grav_cov, time);
  }
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

double ErrorStateKalmanFilter::measureMagneticField3d(
  const Vector3d& mag_meas,
  const Matrix3d& mag_cov,
  const steady_clock::time_point& time)
{
  if (mag_W_.norm() == 0.) {
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

  // 観測方程式 (memo: 3-20)
  H_mag_.block<3, 3>(0, kDeltaThetaIdx) = T * eigen::skew(2 * mag_B);
  H_mag_.block<3, 6>(0, kDeltaMagSoftBiasIdx) << mx, my, mz, 0, 0, 0, 0, mx, 0, my, mz, 0, 0, 0, mx, 0, my, mz;

  // 事後推定を更新
  return correct(delta_mag, mag_cov, H_mag_);
}

double ErrorStateKalmanFilter::measureMagneticFieldYaw(
  const Vector3d& mag_meas,
  const Matrix3d& mag_cov,
  const steady_clock::time_point& time)
{
  if (mag_W_.norm() == 0.) {
    cerr << "Reference magnetic field is not set." << endl;
    return INFINITY;
  }

  const auto& x = x_history_.closestAfterValue(time);

  // オイラー角を取得
  double roll_pred, pitch_pred, yaw_pred;
  const auto R_W_B = getQuaternion(x);
  tobas_std::eulerFromQuaternion(R_W_B.x(), R_W_B.y(), R_W_B.z(), R_W_B.w(), roll_pred, pitch_pred, yaw_pred);

  // 地磁気をヨー角のみ機体と一致し，XY軸が地面と平行な地上座標系Gに移す．
  const AngleAxisd R_W_G(yaw_pred, Vector3d::UnitZ());
  const auto mag_G = R_W_G.inverse() * (R_W_B * mag_meas);  // 後ろから計算することで計算量を削減
  const auto mx = mag_G.x();
  const auto my = mag_G.y();

  // ヨーの誤差を計算
  const auto yaw_ref = atan2(mag_W_.y(), mag_W_.x());
  const auto yaw_meas = yaw_ref - atan2(my, mx);
  const auto delta_yaw = algo::wrapPi(yaw_meas - yaw_pred);

  // 地磁気の分散からヨー角の分散を推定 (memo: 2-75)
  const auto mx_std = sqrt(mag_cov(0, 0));
  const auto my_std = sqrt(mag_cov(1, 1));
  const auto yaw_std = (fabs(mx) * my_std + fabs(my) * mx_std) / (math::sqr(mx) + math::sqr(my));
  const auto yaw_var = math::sqr(yaw_std);

  // 出力方程式を更新
  H_yaw_.block<1, 3>(0, kDeltaThetaIdx) = hamiltonToYawOutputMatrix(x) * getQ_dtheta(x);

  // 事後推定を更新
  return correct(Scalard(delta_yaw), Scalard(yaw_var), H_yaw_);
}

Matrix<double, 4, 3> ErrorStateKalmanFilter::getQ_dtheta(const StateVector& x) const
{
  const Vector4d qby2 = 0.5 * getHamilton(x);
  const auto& qw = qby2(0);
  const auto& qx = qby2(1);
  const auto& qy = qby2(2);
  const auto& qz = qby2(3);
  return (Matrix<double, 4, 3>() << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw).finished();
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
  // Choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw_pred
  constexpr double kEpsilon = 1e-6;
  const Quaterniond q = getQuaternion(x);

  bool can_use_A = false;
  const auto SA0 = 2 * q.z();
  const auto SA1 = 2 * q.y();
  const auto SA2 = SA0 * q.w() + SA1 * q.x();
  const auto SA3 = math::sqr(q.w()) + math::sqr(q.x()) - math::sqr(q.y()) - math::sqr(q.z());
  double SA4, SA5_inv;
  if (math::sqr(SA3) > kEpsilon) {
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
  if (math::sqr(SB2) > kEpsilon) {
    SB3 = 1 / math::sqr(SB2);
    SB5_inv = SB3 * math::sqr(SB4) + 1;
    can_use_B = fabs(SB5_inv) > kEpsilon;
  }

  // Compute output matrix
  RowVector4d H;
  if (can_use_A && (!can_use_B || fabs(SA5_inv) >= fabs(SB5_inv))) {
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
  else if (can_use_B && (!can_use_A || fabs(SB5_inv) > fabs(SA5_inv))) {
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
  else {
    cerr << "Unable to compute the output matrix of yaw_pred angle observation." << endl;
    return RowVector4d::Zero();
  }

  return H;
}

void ErrorStateKalmanFilter::setMagSoftBiasFromMatrix(const Eigen::Matrix3d& T)
{
  x_.segment<6>(kMagSoftBiasIdx) << T(0, 0), T(0, 1), T(0, 2), T(1, 1), T(1, 2), T(2, 2);
}

void ErrorStateKalmanFilter::applyConstraints()
{
  // クオータニオンのノルムは1
  x_.segment<4>(kQuatIdx) = getHamilton().normalized();

  // 事前知識を用いて状態の範囲を制限
  if (acc_bias_proc_noise_density_ > 0.) {
    x_.segment<3>(kAccBiasIdx) = getAccelBias().cwiseMax(-kMaxAccBias).cwiseMin(kMaxAccBias);
  }
  if (gyro_bias_proc_noise_density_ > 0.) {
    x_.segment<3>(kGyroBiasIdx) = getGyroBias().cwiseMax(-kMaxGyroBias).cwiseMin(kMaxGyroBias);
  }
  if (mag_hard_bias_proc_noise_density_ > 0.) {
    x_.segment<3>(kMagHardBiasIdx) = getMagHardBias().cwiseMax(-kMaxMagHardBias).cwiseMin(kMaxMagHardBias);
  }
  if (grav_proc_noise_density_ > 0.) {
    x_(kGravIdx) = clamp(getGravity(), kMinGravity, kMaxGravity);
  }

  // 地磁気のソフトバイアスは正定値対称
  if (mag_soft_bias_proc_noise_density_) {
    const auto T = getMagSoftBias();
    const auto T_positive = eigen::nearestPositiveDefinite(T, kMinMagSoftBiasEigenValue);
    setMagSoftBiasFromMatrix(T_positive);
  }

  // 共分散行列は対称行列でなければならない
  if (enable_cov_symmetrisation_) {
    eigen::symmetrise(P_);
  }
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
