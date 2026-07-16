// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eskf/eskf.hpp"

#include <iostream>

#include <tobas_algorithm/core.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>

#define E3 Eigen::Diagonal3d(1, 1, 1)

namespace ch = std::chrono;

namespace tobas
{
namespace eskf
{
ErrorStateKalmanFilter::ErrorStateKalmanFilter() : x_history_(kStateHistoryTimeWindow)
{
  // Fill the fixed parts of the observation equations.
  H_pos_.setZero();
  H_vel_.setZero();
  H_pv_.setZero();
  H_theta_.setZero();
  H_pose_.setZero();
  H_mag_.setZero();
  H_yaw_.setZero();
  H_baro_alt_.setZero();
  H_grav_.setZero();

  H_pos_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_vel_.block<3, 3>(0, kDeltaVelIdx).diagonal().setOnes();
  H_pv_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_pv_.block<3, 3>(3, kDeltaVelIdx).diagonal().setOnes();
  H_theta_.block<3, 3>(0, kDeltaThetaIdx).diagonal().setOnes();
  H_pose_.block<3, 3>(0, kDeltaPosIdx).diagonal().setOnes();
  H_pose_.block<3, 3>(3, kDeltaThetaIdx).diagonal().setOnes();
  H_mag_.block<3, 3>(0, kDeltaMagHardBiasIdx).diagonal().setOnes();
  H_baro_alt_(0, kDeltaAltIdx) = 1;
  H_baro_alt_(0, kDeltaBaroAltBiasIdx) = 1;
  H_grav_.block<3, 3>(0, kDeltaAccBiasIdx).diagonal().setOnes();
}

void ErrorStateKalmanFilter::initialize(
  const Eigen::Vector3d& init_pos,
  const Eigen::Matrix3d& init_pos_cov,
  const Eigen::Vector3d& init_vel,
  const Eigen::Matrix3d& init_vel_cov,
  const Eigen::Quaterniond& init_quat,
  const Eigen::Matrix3d& init_dtheta_cov,
  const Eigen::Vector3d& init_acc_bias,
  const Eigen::Matrix3d& init_acc_bias_cov,
  const Eigen::Vector3d& init_gyro_bias,
  const Eigen::Matrix3d& init_gyro_bias_cov,
  const Eigen::Vector3d& init_mag_hard_bias,
  const Eigen::Matrix3d& init_mag_hard_bias_cov,
  const Eigen::Matrix3d& init_mag_soft_bias,
  const Eigen::Matrix6d& init_mag_soft_bias_cov,
  const double& init_baro_alt_bias,
  const double& init_baro_alt_bias_var,
  const double& init_grav,
  const double& init_grav_var,
  const ch::steady_clock::time_point& time)
{
  // Set initial IMU time.
  t_last_imu_ = time;

  // Fill the constant part of matrices.
  P_.setZero();
  F_x_.setIdentity();
  G_.setIdentity();

  // Initialize states and covariances.
  initializePosition(init_pos, init_pos_cov);
  initializeVelocity(init_vel, init_vel_cov);
  initializeQuaternion(init_quat, init_dtheta_cov);
  initializeAccelBias(init_acc_bias, init_acc_bias_cov);
  initializeGyroBias(init_gyro_bias, init_gyro_bias_cov);
  initializeMagHardBias(init_mag_hard_bias, init_mag_hard_bias_cov);
  initializeMagSoftBias(init_mag_soft_bias, init_mag_soft_bias_cov);
  initializeBaroAltBias(init_baro_alt_bias, init_baro_alt_bias_var);
  initializeGravity(init_grav, init_grav_var);
}

void ErrorStateKalmanFilter::initializePosition(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(cov));

  x_.segment<3>(kPosIdx) = value;

  P_.middleRows<3>(kDeltaPosIdx).setZero();
  P_.middleCols<3>(kDeltaPosIdx).setZero();
  P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx) = cov;

  resetStateHistory();
}

void ErrorStateKalmanFilter::initializeVelocity(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(cov));

  x_.segment<3>(kVelIdx) = value;

  P_.middleRows<3>(kDeltaVelIdx).setZero();
  P_.middleCols<3>(kDeltaVelIdx).setZero();
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) = cov;

  resetStateHistory();
}

void ErrorStateKalmanFilter::initializeQuaternion(const Eigen::Quaterniond& value, const Eigen::Matrix3d& cov)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(cov));

  x_.segment<4>(kQuatIdx) = eigen::hamiltonFromQuaternion(value).normalized();

  P_.middleRows<3>(kDeltaThetaIdx).setZero();
  P_.middleCols<3>(kDeltaThetaIdx).setZero();
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = cov;

  resetStateHistory();
}

void ErrorStateKalmanFilter::initializeAccelBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(cov));

  x_.segment<3>(kAccBiasIdx) = value;

  P_.middleRows<3>(kDeltaAccBiasIdx).setZero();
  P_.middleCols<3>(kDeltaAccBiasIdx).setZero();
  P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx) = cov;

  resetStateHistory();
}

void ErrorStateKalmanFilter::initializeGyroBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(cov));

  x_.segment<3>(kGyroBiasIdx) = value;

  P_.middleRows<3>(kDeltaGyroBiasIdx).setZero();
  P_.middleCols<3>(kDeltaGyroBiasIdx).setZero();
  P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx) = cov;

  resetStateHistory();
}

void ErrorStateKalmanFilter::initializeMagHardBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(cov));

  x_.segment<3>(kMagHardBiasIdx) = value;

  P_.middleRows<3>(kDeltaMagHardBiasIdx).setZero();
  P_.middleCols<3>(kDeltaMagHardBiasIdx).setZero();
  P_.block<3, 3>(kDeltaMagHardBiasIdx, kDeltaMagHardBiasIdx) = cov;

  resetStateHistory();
}

void ErrorStateKalmanFilter::initializeMagSoftBias(const Eigen::Matrix3d& value, const Eigen::Matrix6d& cov)
{
  assert(eigen::isSymmetricPositiveDefinite(value));
  assert(eigen::isSymmetricSemiPositiveDefinite(cov));

  setMagSoftBiasFromMatrix(value);

  P_.middleRows<6>(kDeltaMagSoftBiasIdx).setZero();
  P_.middleCols<6>(kDeltaMagSoftBiasIdx).setZero();
  P_.block<6, 6>(kDeltaMagSoftBiasIdx, kDeltaMagSoftBiasIdx) = cov;

  resetStateHistory();
}

void ErrorStateKalmanFilter::initializeBaroAltBias(const double& value, const double& var)
{
  assert(var >= 0.0);

  x_(kBaroAltBiasIdx) = value;

  P_.row(kDeltaBaroAltBiasIdx).setZero();
  P_.col(kDeltaBaroAltBiasIdx).setZero();
  P_(kDeltaBaroAltBiasIdx, kDeltaBaroAltBiasIdx) = var;

  resetStateHistory();
}

void ErrorStateKalmanFilter::initializeGravity(const double& value, const double& var)
{
  assert(var >= 0.0);

  x_(kGravIdx) = value;

  P_.row(kDeltaGravIdx).setZero();
  P_.col(kDeltaGravIdx).setZero();
  P_(kDeltaGravIdx, kDeltaGravIdx) = var;

  resetStateHistory();
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
  if (value < 0.0) {
    std::cerr << "The noise density of accelerometer bias process must be non-negative." << std::endl;
    return false;
  }

  acc_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setGyroBiasProcNoiseDensity(double value)
{
  if (value < 0.0) {
    std::cerr << "The noise density of gyroscope bias process must be non-negative." << std::endl;
    return false;
  }

  gyro_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setMagHardBiasProcNoiseDensity(double value)
{
  if (value < 0.0) {
    std::cerr << "The noise density of magnetometer hard-iron bias process must be non-negative." << std::endl;
    return false;
  }

  mag_hard_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setMagSoftBiasProcNoiseDensity(double value)
{
  if (value < 0.0) {
    std::cerr << "The noise density of magnetometer soft-iron bias process must be non-negative." << std::endl;
    return false;
  }

  mag_soft_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setBaroAltBiasProcNoiseDensity(double value)
{
  if (value < 0.0) {
    std::cerr << "The noise density of barometer altitude bias process must be non-negative." << std::endl;
    return false;
  }

  baro_alt_bias_proc_noise_density_ = value;
  return true;
}

bool ErrorStateKalmanFilter::setGravProcNoiseDensity(double value)
{
  if (value < 0.0) {
    std::cerr << "The noise density of gravity process must be non-negative." << std::endl;
    return false;
  }

  grav_proc_noise_density_ = value;
  return true;
}

void ErrorStateKalmanFilter::setMagneticFieldRef(const Eigen::Vector3d& mag_W)
{
  assert(mag_W.norm() > 0.0);
  mag_W_ = mag_W.normalized();
}

void ErrorStateKalmanFilter::setAirPressureOrigin(double pres)
{
  baro_alt_origin_ = st::pressureToAltitude(pres);
}

double ErrorStateKalmanFilter::measureIMU(
  const Eigen::Vector3d& acc_meas,
  const Eigen::Vector3d& gyro_meas,
  const Eigen::Matrix3d& acc_cov,
  const Eigen::Matrix3d& gyro_cov,
  const Eigen::Matrix3d& grav_cov,
  const ch::steady_clock::time_point& time)
{
  assert(eigen::isSymmetricSemiPositiveDefinite(acc_cov));
  assert(eigen::isSymmetricSemiPositiveDefinite(gyro_cov));
  assert(eigen::isSymmetricPositiveDefinite(grav_cov));

  // Compute the sampling time and update the timestamp.
  const auto dt = ch::duration<double>(time - t_last_imu_).count();  // [s]
  const auto dt2 = math::sqr(dt);
  t_last_imu_ = time;

  // `dt = 0` cannot be accepted because quaternion normalization depends on it.
  if (dt <= 0.0) {
    std::cerr << "IMU time gap must be positive: " << dt << " <= 0 [sec]" << std::endl;
    return INFINITY;
  }

  const Eigen::Vector3d acc_B = acc_meas - getAccelBias(x_);
  const Eigen::Vector3d gyro_B = gyro_meas - getGyroBias(x_);

  const Eigen::Quaterniond delta_q = eigen::quaternionFromAngleAxis(gyro_B * dt);
  const Eigen::Matrix3d delta_R = delta_q.toRotationMatrix();

  const Eigen::Vector3d vel_W = getVelocity(x_);
  const Eigen::Quaterniond q = getQuaternion(x_);
  const Eigen::Matrix3d W_Rot_B = q.toRotationMatrix();
  const Eigen::Vector3d acc_grav_W = W_Rot_B * acc_B + getGravVector(x_);

  // Correct IMU covariance.
  const auto acc_cov_fixed = eigen::nearestPositiveDefinite(acc_cov, math::sqr(kMinAccStddev));
  const auto gyro_cov_fixed = eigen::nearestPositiveDefinite(gyro_cov, math::sqr(kMinGyroStddev));

  // (260) Nominal-state kinematics.
  x_.segment<3>(kPosIdx) += vel_W * dt;
  if (enable_second_integral_) {
    x_.segment<3>(kPosIdx) += 0.5 * acc_grav_W * dt2;  // Risk of increased integration error.
  }
  x_.segment<3>(kVelIdx) += acc_grav_W * dt;
  x_.segment<4>(kQuatIdx) = eigen::hamiltonFromQuaternion(q * delta_q);

  // (270) Update the variable part of the Jacobian.
  F_x_.block<3, 3>(kDeltaPosIdx, kDeltaVelIdx).diagonal().fill(dt);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaThetaIdx) = -W_Rot_B * eigen::skew(acc_B * dt);
  F_x_.block<3, 3>(kDeltaVelIdx, kDeltaAccBiasIdx) = -W_Rot_B * dt;
  F_x_(kDeltaVelIdx + 2, kDeltaGravIdx) = -dt;
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = delta_R.transpose();
  F_x_.block<3, 3>(kDeltaThetaIdx, kDeltaGyroBiasIdx).diagonal().fill(-dt);

  // (269) First term: update the predicted covariance matrix.
  P_ = F_x_ * P_.selfadjointView<Eigen::Lower>() * F_x_.transpose();  // TODO: Compute only the required parts.

  // (269) Second term: apply process noise.
  P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx) += W_Rot_B * acc_cov_fixed * W_Rot_B.transpose() * dt2;
  P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) += W_Rot_B * gyro_cov_fixed * W_Rot_B.transpose() * dt2;
  P_.diagonal().segment<3>(kDeltaAccBiasIdx).array() += math::sqr(acc_bias_proc_noise_density_) * dt;
  P_.diagonal().segment<3>(kDeltaGyroBiasIdx).array() += math::sqr(gyro_bias_proc_noise_density_) * dt;
  P_.diagonal().segment<3>(kDeltaMagHardBiasIdx).array() += math::sqr(mag_hard_bias_proc_noise_density_) * dt;
  P_.diagonal().segment<6>(kDeltaMagSoftBiasIdx).array() += math::sqr(mag_soft_bias_proc_noise_density_) * dt;
  P_(kDeltaBaroAltBiasIdx, kDeltaBaroAltBiasIdx) += math::sqr(baro_alt_bias_proc_noise_density_) * dt;
  P_(kDeltaGravIdx, kDeltaGravIdx) += math::sqr(grav_proc_noise_density_) * dt;

  // Apply constraints to avoid numerical errors.
  applyConstraints();

  // Save the state history.
  x_history_.add(time, x_);

  // Gravity-direction observation: acceleration and attitude have an equality relation (= output equation),
  // so correction is performed according to Kalman filter theory.
  // During free fall or when acceleration is too large, the acceleration may not reflect attitude at all,
  // so the gravity-direction observation is limited to acceleration values in this range.
  const auto acc_norm = acc_meas.norm();
  const auto gravity = getGravity(x_);
  if (acc_norm < kFreeFallAccelNormThresh * gravity) {
    std::cerr << "Attitude correction cannot be performed because the aircraft is in free fall." << std::endl;
    return INFINITY;
  }

  return measureGravity(acc_meas, grav_cov, time);
}

double ErrorStateKalmanFilter::measurePosition(
  const Eigen::Vector3d& pos_meas,
  const Eigen::Matrix3d& pos_cov,
  const Eigen::Vector3d& offset,
  const ch::steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Eigen::Vector3d delta = pos_meas - getPosition(x, offset);

  // Partial derivative with respect to attitude.
  const auto dqvq_dq = quatRotationDerivative(x, offset);
  const auto Q_dtheta = getQ_dtheta(x);
  H_pos_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  return correct(delta, pos_cov, H_pos_);
}

double ErrorStateKalmanFilter::measureVelocity(
  const Eigen::Vector3d& vel_meas,
  const Eigen::Matrix3d& vel_cov,
  const Eigen::Vector3d& offset,
  const Eigen::Vector3d& gyro_meas,
  const ch::steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const Eigen::Vector3d gyro_nominal = gyro_meas - getGyroBias(x);
  const Eigen::Vector3d gyro_offset = gyro_nominal.cross(offset);
  const Eigen::Vector3d vel_nominal = getVelocity(x) + getQuaternion(x) * gyro_offset;
  const Eigen::Vector3d delta = vel_meas - vel_nominal;

  // Partial derivative with respect to attitude.
  const auto dqvq_dq = quatRotationDerivative(x, gyro_offset);
  const auto Q_dtheta = getQ_dtheta(x);
  H_vel_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  // Partial derivative with respect to gyro bias.
  H_vel_.block<3, 3>(0, kDeltaGyroBiasIdx) = getDCM(x) * eigen::skew(offset);

  return correct(delta, vel_cov, H_vel_);
}

double ErrorStateKalmanFilter::measurePosVel(
  const Eigen::Vector3d& pos_meas,
  const Eigen::Vector3d& vel_meas,
  const Eigen::Matrix6d& cov,
  const Eigen::Vector3d& offset,
  const Eigen::Vector3d& gyro_meas,
  const ch::steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  // Observation error.
  Eigen::Vector6d delta;
  const Eigen::Vector3d gyro_nominal = gyro_meas - getGyroBias(x);
  const Eigen::Vector3d gyro_offset = gyro_nominal.cross(offset);
  const Eigen::Vector3d vel_nominal = getVelocity(x) + getQuaternion(x) * gyro_offset;
  delta.head<3>() = pos_meas - getPosition(x, offset);  // Position error.
  delta.tail<3>() = vel_meas - vel_nominal;             // Velocity error.

  // Observation equation.
  const auto Q_dtheta = getQ_dtheta(x);
  const auto pos_q_deriv = quatRotationDerivative(x, offset);
  const auto vel_q_deriv = quatRotationDerivative(x, gyro_offset);
  H_pv_.block<3, 3>(0, kDeltaThetaIdx) = pos_q_deriv * Q_dtheta;  // Position derivative by attitude.
  H_pv_.block<3, 3>(3, kDeltaThetaIdx) = vel_q_deriv * Q_dtheta;  // Velocity derivative by attitude.
  H_pv_.block<3, 3>(0, kDeltaGyroBiasIdx) = getDCM(x) * eigen::skew(offset);

  // Update the posterior estimate.
  return correct(delta, cov, H_pv_);
}

double ErrorStateKalmanFilter::measureQuaternion(
  const Eigen::Quaterniond& q_meas,
  const Eigen::Matrix3d& theta_cov,
  const ch::steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const auto q_error = getQuaternion(x).conjugate() * q_meas;
  const auto delta = eigen::angleAxisFromQuaternion(q_error);

  return correct(delta, theta_cov, H_theta_);
}

double ErrorStateKalmanFilter::measurePose(
  const Eigen::Vector3d& pos_meas,
  const Eigen::Quaterniond& q_meas,
  const Eigen::Matrix6d& cov,
  const Eigen::Vector3d& offset,
  const ch::steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  // Observation error.
  Eigen::Vector6d delta;
  delta.head<3>() = pos_meas - getPosition(x, offset);
  delta.tail<3>() = eigen::angleAxisFromQuaternion(getQuaternion(x).conjugate() * q_meas);

  // Partial derivative of the position observation with respect to attitude.
  const auto dqvq_dq = quatRotationDerivative(x, offset);
  const auto Q_dtheta = getQ_dtheta(x);
  H_pose_.block<3, 3>(0, kDeltaThetaIdx) = dqvq_dq * Q_dtheta;

  return correct(delta, cov, H_pose_);
}

double ErrorStateKalmanFilter::measureMagneticField3d(
  const Eigen::Vector3d& mag_meas,
  const Eigen::Matrix3d& mag_cov,
  const ch::steady_clock::time_point& time)
{
  if (mag_W_.norm() == 0.0) {
    std::cerr << "Reference magnetic field is not set." << std::endl;
    return INFINITY;
  }

  const auto& x = x_history_.closestAfterValue(time);

  const Eigen::Vector3d mag_B = getQuaternion(x).inverse() * mag_W_;
  const Eigen::Vector3d m_b = getMagHardBias(x);
  const Eigen::Matrix3d T = getMagSoftBias(x);

  const auto& mx = mag_B.x();
  const auto& my = mag_B.y();
  const auto& mz = mag_B.z();

  // Observation error.
  const auto mag_pred = T * mag_B + m_b;
  const Eigen::Vector3d delta = mag_meas - mag_pred;

  // Observation equation (memo: 3-20).
  H_mag_.block<3, 3>(0, kDeltaThetaIdx) = T * eigen::skew(2 * mag_B);
  H_mag_.block<3, 6>(0, kDeltaMagSoftBiasIdx) << mx, my, mz, 0, 0, 0, 0, mx, 0, my, mz, 0, 0, 0, mx, 0, my, mz;

  // Update the posterior estimate.
  return correct(delta, mag_cov, H_mag_);
}

double ErrorStateKalmanFilter::measureMagneticFieldHead(
  const Eigen::Vector3d& mag_meas,
  const double& yaw_var,
  const ch::steady_clock::time_point& time)
{
  if (mag_W_.norm() == 0.0) {
    std::cerr << "Reference magnetic field is not set." << std::endl;
    return INFINITY;
  }

  const auto& x = x_history_.closestAfterValue(time);

  // Get Euler angles.
  const auto R_W_B = getQuaternion(x);
  const auto [roll_pred, pitch_pred, yaw_pred] = st::eulerFromQuaternion(R_W_B.x(), R_W_B.y(), R_W_B.z(), R_W_B.w());

  // Move the geomagnetic field to ground coordinate system `G`,
  // whose yaw alone matches the body and whose XY axes are parallel to the ground.
  const Eigen::AngleAxisd R_W_G(yaw_pred, Eigen::Vector3d::UnitZ());
  const auto mag_G = R_W_G.inverse() * (R_W_B * mag_meas);  // Reduce computation by evaluating from the back.
  const auto mx = mag_G.x();
  const auto my = mag_G.y();

  // Compute yaw error.
  const auto yaw_ref = std::atan2(mag_W_.y(), mag_W_.x());
  const auto yaw_meas = yaw_ref - std::atan2(my, mx);
  const auto delta = algo::wrapPi(yaw_meas - yaw_pred);

  // Update the output equation.
  H_yaw_.block<1, 3>(0, kDeltaThetaIdx) = hamiltonToYawOutputMatrix(x) * getQ_dtheta(x);

  // Update the posterior estimate.
  return correct(Eigen::Scalard(delta), Eigen::Scalard(yaw_var), H_yaw_);
}

double ErrorStateKalmanFilter::measureAirPressure(
  const double& pres,
  const double& alt_var,
  const ch::steady_clock::time_point& time)
{
  const auto& x = x_history_.closestAfterValue(time);

  const auto baro_alt_pred = getAltitude(x) + getBaroAltBias(x);
  const auto baro_alt_meas = st::pressureToAltitude(pres) - baro_alt_origin_;
  const auto delta = baro_alt_meas - baro_alt_pred;

  return correct(Eigen::Scalard(delta), Eigen::Scalard(alt_var), H_baro_alt_);
}

Eigen::Matrix<double, 4, 3> ErrorStateKalmanFilter::getQ_dtheta(const StateVector& x) const
{
  const Eigen::Vector4d qby2 = 0.5 * getHamilton(x);
  const auto& qw = qby2(0);
  const auto& qx = qby2(1);
  const auto& qy = qby2(2);
  const auto& qz = qby2(3);
  return (Eigen::Matrix<double, 4, 3>() << -qx, -qy, -qz, qw, -qz, qy, qz, qw, -qx, -qy, qx, qw).finished();
}

Eigen::Matrix<double, 3, 4>
ErrorStateKalmanFilter::quatRotationDerivative(const StateVector& x, const Eigen::Vector3d& a) const
{
  const auto ham = getHamilton(x);
  const auto w = ham(0);
  const auto v = ham.tail<3>();

  Eigen::Matrix<double, 3, 4> res;
  res.block<3, 1>(0, 0) = 2 * (w * a - a.cross(v));
  res.block<3, 3>(0, 1) = 2 * (a.dot(v) * E3 + v * a.transpose() - a * v.transpose() - w * eigen::skew(a));

  return res;
}

Eigen::RowVector4d ErrorStateKalmanFilter::hamiltonToYawOutputMatrix(const StateVector& x) const
{
  // cf. Ekf::fuseYaw321: https://github.com/PX4/PX4-ECL/blob/46dd05a9159817035dab6acebc33f8a3da69d3a7/EKF/mag_fusion.cpp#L420

  constexpr double kEpsilon = 1e-6;

  const auto ham = getHamilton(x);
  const auto& qw = ham(0);
  const auto& qx = ham(1);
  const auto& qy = ham(2);
  const auto& qz = ham(3);

  // Choose A or B computational paths to avoid singularity in derivation at +-90 degrees yaw.
  bool can_use_a = false;
  const auto sa0 = 2 * qz;
  const auto sa1 = 2 * qy;
  const auto sa2 = sa0 * qw + sa1 * qx;
  const auto sa3 = math::sqr(qw) + math::sqr(qx) - math::sqr(qy) - math::sqr(qz);
  double sa4, sa5_inv;
  if (math::sqr(sa3) > kEpsilon) {
    sa4 = 1 / math::sqr(sa3);
    sa5_inv = math::sqr(sa2) * sa4 + 1;
    can_use_a = std::abs(sa5_inv) > kEpsilon;
  }
  bool can_use_b = false;
  const auto sb0 = 2 * qw;
  const auto sb1 = 2 * qx;
  const auto sb2 = sb0 * qz + sb1 * qy;
  const auto sb4 = math::sqr(qw) + math::sqr(qx) - math::sqr(qy) - math::sqr(qz);
  double sb3, sb5_inv;
  if (math::sqr(sb2) > kEpsilon) {
    sb3 = 1 / math::sqr(sb2);
    sb5_inv = sb3 * math::sqr(sb4) + 1;
    can_use_b = std::abs(sb5_inv) > kEpsilon;
  }

  // Compute output matrix.
  Eigen::RowVector4d H;
  if (can_use_a && (!can_use_b || std::abs(sa5_inv) >= std::abs(sb5_inv))) {
    const auto sa5 = 1 / sa5_inv;
    const auto sa6 = 1 / sa3;
    const auto sa7 = sa2 * sa4;
    const auto sa8 = 2 * sa7;
    const auto sa9 = 2 * sa6;

    H(0) = sa5 * (sa0 * sa6 - sa8 * qw);
    H(1) = sa5 * (sa1 * sa6 - sa8 * qx);
    H(2) = sa5 * (sa1 * sa7 + sa9 * qx);
    H(3) = sa5 * (sa0 * sa7 + sa9 * qw);
  }
  else if (can_use_b && (!can_use_a || std::abs(sb5_inv) > std::abs(sa5_inv))) {
    const auto sb5 = 1 / sb5_inv;
    const auto sb6 = 1 / sb2;
    const auto sb7 = sb3 * sb4;
    const auto sb8 = 2 * sb7;
    const auto sb9 = 2 * sb6;

    H(0) = -sb5 * (sb0 * sb6 - sb8 * qz);
    H(1) = -sb5 * (sb1 * sb6 - sb8 * qy);
    H(2) = -sb5 * (-sb1 * sb7 - sb9 * qy);
    H(3) = -sb5 * (-sb0 * sb7 - sb9 * qz);
  }
  else {
    std::cerr << "Unable to compute the output matrix of yaw angle observation." << std::endl;
    return Eigen::RowVector4d::Zero();
  }

  return H;
}

void ErrorStateKalmanFilter::setMagSoftBiasFromMatrix(const Eigen::Matrix3d& T)
{
  x_.segment<6>(kMagSoftBiasIdx) << T(0, 0), T(0, 1), T(0, 2), T(1, 1), T(1, 2), T(2, 2);
}

void ErrorStateKalmanFilter::applyConstraints()
{
  // Quaternion norm is one.
  x_.segment<4>(kQuatIdx) = getHamilton().normalized();

  // Limit the state range using prior knowledge.
  if (acc_bias_proc_noise_density_ > 0.0) {
    x_.segment<3>(kAccBiasIdx) = getAccelBias().cwiseMax(-kMaxAccBias).cwiseMin(kMaxAccBias);
  }
  if (gyro_bias_proc_noise_density_ > 0.0) {
    x_.segment<3>(kGyroBiasIdx) = getGyroBias().cwiseMax(-kMaxGyroBias).cwiseMin(kMaxGyroBias);
  }
  if (mag_hard_bias_proc_noise_density_ > 0.0) {
    x_.segment<3>(kMagHardBiasIdx) = getMagHardBias().cwiseMax(-kMaxMagHardBias).cwiseMin(kMaxMagHardBias);
  }
  if (grav_proc_noise_density_ > 0.0) {
    x_(kGravIdx) = std::clamp(getGravity(), kMinGravity, kMaxGravity);
  }

  // Magnetometer soft-iron bias is symmetric positive definite.
  if (mag_soft_bias_proc_noise_density_) {
    const auto T = getMagSoftBias();
    const auto T_positive = eigen::nearestPositiveDefinite(T, kMinMagSoftBiasEigenValue);
    setMagSoftBiasFromMatrix(T_positive);
  }

  // The covariance matrix must be symmetric.
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
  const Eigen::Vector3d& acc_meas,
  const Eigen::Matrix3d& grav_cov,
  const ch::steady_clock::time_point& time)
{
  // Observation equation:
  //    a_t = a_b - R^T g (g = [0, 0, -9.8xxxx])
  // Jacobian:
  //    pda_t/pdq = -2 [R^T g]x
  //    pda_t/pda_b = E_3
  //    pda_t/pdg = -R^T (the sign is inverted to estimate only positive gravitational acceleration,
  //    and only the second column is used).

  const auto& x = x_history_.closestAfterValue(time);

  const Eigen::Matrix3d R_B_W = getDCM(x).transpose();
  const Eigen::Vector3d grav_B = R_B_W * getGravVector(x);
  const Eigen::Vector3d acc_ref = getAccelBias(x) - grav_B;  // Acceleration expected without dynamic acceleration.
  const Eigen::Vector3d delta = acc_meas - acc_ref;

  H_grav_.block<3, 3>(0, kDeltaThetaIdx) = -eigen::skew(2 * grav_B);
  H_grav_.col(kDeltaGravIdx) = R_B_W.col(2);
  return correct(delta, grav_cov, H_grav_);
}
}  // namespace eskf
}  // namespace tobas
