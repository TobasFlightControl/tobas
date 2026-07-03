// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_std_tools/timestamped_buffer.hpp>

namespace tobas
{
namespace eskf
{
/**
 * @brief Error-state Kalman filter.
 *
 * Base algorithm: Quaternion kinematics for the error-state Kalman filter [Sola, 2017]
 * Japanese translation: https://www.flight.t.u-tokyo.ac.jp/?p=800
 *
 * Magnetometer bias estimation extension: Online 3-Axis Magnetometer Hard-Iron and Soft-Iron Bias and Angular Velocity
 * Sensor Bias Estimation Using Angular Velocity Sensors for Improved Dynamic Heading Accuracy [Spielvogel+, 2022]
 *
 * @note Formulated in the IMU frame.
 */
class ErrorStateKalmanFilter
{
  // Nominal state indices.
  static constexpr size_t kPosIdx = 0;
  static constexpr size_t kAltIdx = kPosIdx + 2;
  static constexpr size_t kVelIdx = kPosIdx + 3;
  static constexpr size_t kQuatIdx = kVelIdx + 3;
  static constexpr size_t kAccBiasIdx = kQuatIdx + 4;
  static constexpr size_t kGyroBiasIdx = kAccBiasIdx + 3;
  static constexpr size_t kMagHardBiasIdx = kGyroBiasIdx + 3;
  static constexpr size_t kMagSoftBiasIdx = kMagHardBiasIdx + 3;
  static constexpr size_t kBaroAltBiasIdx = kMagSoftBiasIdx + 6;
  static constexpr size_t kGravIdx = kBaroAltBiasIdx + 1;
  static constexpr size_t kStateSize = kGravIdx + 1;

  // Error-state indices.
  static constexpr size_t kDeltaPosIdx = 0;
  static constexpr size_t kDeltaAltIdx = kDeltaPosIdx + 2;
  static constexpr size_t kDeltaVelIdx = kDeltaPosIdx + 3;
  static constexpr size_t kDeltaThetaIdx = kDeltaVelIdx + 3;
  static constexpr size_t kDeltaAccBiasIdx = kDeltaThetaIdx + 3;
  static constexpr size_t kDeltaGyroBiasIdx = kDeltaAccBiasIdx + 3;
  static constexpr size_t kDeltaMagHardBiasIdx = kDeltaGyroBiasIdx + 3;
  static constexpr size_t kDeltaMagSoftBiasIdx = kDeltaMagHardBiasIdx + 3;
  static constexpr size_t kDeltaBaroAltBiasIdx = kDeltaMagSoftBiasIdx + 6;
  static constexpr size_t kDeltaGravIdx = kDeltaBaroAltBiasIdx + 1;
  static constexpr size_t kDeltaStateSize = kDeltaGravIdx + 1;

  // Variable ranges.
  static constexpr double kMaxAccBias = 1.;                 // [m/s^2]
  static constexpr double kMaxGyroBias = 0.1;               // [rad/s]
  static constexpr double kMaxMagHardBias = 2.;             // [-]
  static constexpr double kMinMagSoftBiasEigenValue = 0.1;  // [-]
  static constexpr double kMinGravity = 9.75;               // [m/s^2]
  static constexpr double kMaxGravity = 9.85;               // [m/s^2]

  // Limits on sensor uncertainty.
  // Each sensor reading includes bias, so an extremely small variance observed at rest or in similar conditions,
  // where the bias component is likely dominant, does not represent the real uncertainty.
  // If a biased value is treated as certain and the covariance does not grow,
  // observation correction may not be applied and the attitude may diverge,
  // so lower bounds are imposed on the eigenvalues.
  static constexpr double kMinGyroStddev = 0.01;  // [rad/s]
  static constexpr double kMinAccStddev = 0.1;    // [m/s^2]

  // Miscellaneous.
  static constexpr auto kStateHistoryTimeWindow = std::chrono::milliseconds(500);
  static constexpr double kFreeFallAccelNormThresh = 0.1;  // [G]

  using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using StateVector = Eigen::Vector<double, kStateSize>;
  using RowStateVector = Eigen::RowVector<double, kStateSize>;
  using DeltaStateMatrix = Eigen::Matrix<double, kDeltaStateSize, kDeltaStateSize>;
  using DeltaStateVector = Eigen::Vector<double, kDeltaStateSize>;
  using RowDeltaStateVector = Eigen::RowVector<double, kDeltaStateSize>;

public:
  explicit ErrorStateKalmanFilter();

  void initialize(
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
    const std::chrono::steady_clock::time_point& time);

  void initializePosition(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  void initializeVelocity(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  void initializeQuaternion(const Eigen::Quaterniond& value, const Eigen::Matrix3d& cov);
  void initializeAccelBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  void initializeGyroBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  void initializeMagHardBias(const Eigen::Vector3d& value, const Eigen::Matrix3d& cov);
  void initializeMagSoftBias(const Eigen::Matrix3d& value, const Eigen::Matrix6d& cov);
  void initializeBaroAltBias(const double& value, const double& var);
  void initializeGravity(const double& value, const double& var);

  void enableSecondIntegral(bool enable);
  void enableCovSymmetrisation(bool enable);
  void enableCovInitialization(bool enable);
  void enableJosephForm(bool enable);

  bool setAccBiasProcNoiseDensity(double value);      // [m/s^3/√Hz]
  bool setGyroBiasProcNoiseDensity(double value);     // [rad/s^2/√Hz]
  bool setMagHardBiasProcNoiseDensity(double value);  // [/s/√Hz]
  bool setMagSoftBiasProcNoiseDensity(double value);  // [/s/√Hz]
  bool setBaroAltBiasProcNoiseDensity(double value);  // [m/s/√Hz]
  bool setGravProcNoiseDensity(double value);         // [m/s^3/√Hz]

  void setMagneticFieldRef(const Eigen::Vector3d& mag_W);
  void setAirPressureOrigin(double pres);

  // Direct value getters
  inline Eigen::Vector3d getPosition() const;
  inline Eigen::Vector3d getVelocity() const;
  inline Eigen::Vector4d getHamilton() const;
  inline Eigen::Vector3d getAccelBias() const;
  inline Eigen::Vector3d getGyroBias() const;
  inline Eigen::Vector3d getMagHardBias() const;
  inline Eigen::Matrix3d getMagSoftBias() const;
  inline double getBaroAltBias() const;
  inline double getGravity() const;

  // Extended value getters
  inline Eigen::Quaterniond getQuaternion() const;

  inline Eigen::Matrix3d getPositionCovariance() const;
  inline Eigen::Matrix3d getVelocityCovariance() const;
  inline Eigen::Matrix3d getRotationCovariance() const;
  inline Eigen::Matrix3d getAccelBiasCovariance() const;
  inline Eigen::Matrix3d getGyroBiasCovariance() const;
  inline Eigen::Matrix3d getMagHardBiasCovariance() const;
  inline Eigen::Matrix6d getMagSoftBiasCovariance() const;
  inline double getGravityVariance() const;
  inline double getBaroAltBiasVariance() const;

  /**
   * @brief Predict the next state from acceleration and gyro measurements, and correct the attitude.
   *
   * @param acc_meas [m/s^2] Measured acceleration.
   * @param gyro_meas [rad/s] Measured gyro value.
   * @param acc_cov [m^2/s^4] Covariance of acceleration measurement noise.
   * @param gyro_cov [rad^2/s^2] Covariance of gyro measurement noise.
   * @param grav_cov [m^2/s^4] Covariance of gravitational acceleration measurement noise.
   * @param time [s] Current time.
   */
  double measureIMU(
    const Eigen::Vector3d& acc_meas,
    const Eigen::Vector3d& gyro_meas,
    const Eigen::Matrix3d& acc_cov,
    const Eigen::Matrix3d& gyro_cov,
    const Eigen::Matrix3d& grav_cov,
    const std::chrono::steady_clock::time_point& time);

  /**
   * @brief Apply a position observation to the nominal state.
   *
   * @param pos_meas Position observation expressed in the world coordinate system.
   * @param pos_cov Covariance of position measurement noise.
   * @param offset Offset of the observation frame from the IMU frame, expressed in the IMU frame.
   *
   * @return Anomaly score
   */
  double measurePosition(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& offset,
    const std::chrono::steady_clock::time_point& time);

  /**
   * @brief Apply a velocity observation to the nominal state.
   *
   * @param vel_meas Velocity observation expressed in the world coordinate system.
   * @param vel_cov Covariance of velocity measurement noise.
   * @param offset Offset of the observation frame from the IMU frame, expressed in the IMU frame.
   * @param gyro_meas Gyro sensor reading.
   *
   * @return Anomaly score
   */
  double measureVelocity(
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas,
    const std::chrono::steady_clock::time_point& time);

  double measurePosVel(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix6d& cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas,
    const std::chrono::steady_clock::time_point& time);

  double measureQuaternion(
    const Eigen::Quaterniond& q_meas,
    const Eigen::Matrix3d& theta_cov,
    const std::chrono::steady_clock::time_point& time);

  double measurePose(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Quaterniond& q_meas,
    const Eigen::Matrix6d& cov,
    const Eigen::Vector3d& offset,
    const std::chrono::steady_clock::time_point& time);

  double measureMagneticField3d(
    const Eigen::Vector3d& mag_meas,
    const Eigen::Matrix3d& mag_cov,
    const std::chrono::steady_clock::time_point& time);

  double measureMagneticFieldHead(
    const Eigen::Vector3d& mag_meas,
    const double& yaw_var,
    const std::chrono::steady_clock::time_point& time);

  double
  measureAirPressure(const double& pres, const double& alt_var, const std::chrono::steady_clock::time_point& time);

private:
  // Configuration
  bool enable_second_integral_ = false;
  bool enable_cov_symmetrisation_ = false;
  bool enable_cov_initialization_ = false;
  bool enable_joseph_form_ = true;
  double acc_bias_proc_noise_density_ = 0.;       // [m/s^3/√Hz] Process noise density of acceleration bias.
  double gyro_bias_proc_noise_density_ = 0.;      // [rad/s^2/√Hz] Process noise density of gyro bias.
  double mag_hard_bias_proc_noise_density_ = 0.;  // [/s/√Hz] Process noise density of magnetometer hard-iron bias.
  double mag_soft_bias_proc_noise_density_ = 0.;  // [/s/√Hz] Process noise density of magnetometer soft-iron bias.
  double baro_alt_bias_proc_noise_density_ = 0.;  // [m/s/√Hz] Process noise density of barometric altitude bias.
  double grav_proc_noise_density_ = 0.;           // [m/s^3/√Hz] Process noise density of gravitational acceleration.

  StateVector x_;         // State vector of the filter
  DeltaStateMatrix P_;    // Covariance of the error state
  DeltaStateMatrix F_x_;  // Jacobian of the state transition
  DeltaStateMatrix G_;    // Jacobian of the error initialization

  // Output matrices.
  Eigen::Matrix<double, 3, kDeltaStateSize> H_pos_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_vel_;
  Eigen::Matrix<double, 6, kDeltaStateSize> H_pv_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_theta_;
  Eigen::Matrix<double, 6, kDeltaStateSize> H_pose_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_mag_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_yaw_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_baro_alt_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_grav_;

  std::chrono::steady_clock::time_point t_last_imu_;
  st::TimestampedBuffer<StateVector> x_history_;
  Eigen::Vector3d mag_W_ = Eigen::Vector3d::Zero();
  double baro_alt_origin_ = 0.;

  // Direct value getters
  inline Eigen::Vector3d getPosition(const StateVector& x) const;
  inline Eigen::Vector3d getVelocity(const StateVector& x) const;
  inline Eigen::Vector4d getHamilton(const StateVector& x) const;
  inline Eigen::Vector3d getAccelBias(const StateVector& x) const;
  inline Eigen::Vector3d getGyroBias(const StateVector& x) const;
  inline Eigen::Vector3d getMagHardBias(const StateVector& x) const;
  inline Eigen::Matrix3d getMagSoftBias(const StateVector& x) const;
  inline double getBaroAltBias(const StateVector& x) const;
  inline double getGravity(const StateVector& x) const;

  // Extended value getters
  inline Eigen::Vector2d getXY(const StateVector& x) const;
  inline double getAltitude(const StateVector& x) const;
  inline Eigen::Vector3d getPosition(const StateVector& x, const Eigen::Vector3d& offset) const;
  inline Eigen::Vector3d
  getVelocity(const StateVector& x, const Eigen::Vector3d& offset, const Eigen::Vector3d& gyro_meas) const;
  inline Eigen::Quaterniond getQuaternion(const StateVector& x) const;
  inline Eigen::Matrix3d getDCM(const StateVector& x) const;
  inline Eigen::Vector3d getGravVector(const StateVector& x) const;

  /* (281) */
  Eigen::Matrix<double, 4, 3> getQ_dtheta(const StateVector& x) const;

  /* Partial derivative of vector `v` rotated by `q` with respect to `q`: `d(q * v * q') / d(q)`. */
  Eigen::Matrix<double, 3, 4> quatRotationDerivative(const StateVector& x, const Eigen::Vector3d& a) const;

  /* Output equation from quaternion to yaw. */
  Eigen::RowVector4d hamiltonToYawOutputMatrix(const StateVector& x) const;

  void setMagSoftBiasFromMatrix(const Eigen::Matrix3d& T);
  void applyConstraints();
  void resetStateHistory();

  /**
   * @brief Observation of the gravity direction, used to correct the attitude.
   *
   * @param acc_meas Accelerometer reading.
   * @param grav_cov Parameter that controls the correction amount from the observation.
   * It is mathematically treated as covariance,
   * but it should be tuned because the generally correct value is unknown;
   * in addition to sensor noise, the variance of the estimated attitude also affects it.
   *
   * @return Anomaly score
   */
  double measureGravity(
    const Eigen::Vector3d& acc_meas,
    const Eigen::Matrix3d& grav_cov,
    const std::chrono::steady_clock::time_point& time);

  /**
   * @brief Compute posterior estimates of the state and covariance from an observation.
   *
   * @tparam M Observation dimension.
   * @param delta_meas Error between the observation and the nominal state.
   * @param meas_cov Covariance of observation noise.
   * @param H Observation equation.
   *
   * @return Anomaly score
   */
  template <int M>
  double correct(
    const Eigen::Matrix<double, M, 1>& delta_meas,
    const Eigen::Matrix<double, M, M>& meas_cov,
    const Eigen::Matrix<double, M, kDeltaStateSize>& H);
};

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition() const
{
  return getPosition(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity() const
{
  return getVelocity(x_);
}

inline Eigen::Vector4d ErrorStateKalmanFilter::getHamilton() const
{
  return getHamilton(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getAccelBias() const
{
  return getAccelBias(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGyroBias() const
{
  return getGyroBias(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getMagHardBias() const
{
  return getMagHardBias(x_);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getMagSoftBias() const
{
  return getMagSoftBias(x_);
}

inline double ErrorStateKalmanFilter::getBaroAltBias() const
{
  return getBaroAltBias(x_);
}

inline double ErrorStateKalmanFilter::getGravity() const
{
  return getGravity(x_);
}

inline Eigen::Quaterniond ErrorStateKalmanFilter::getQuaternion() const
{
  return getQuaternion(x_);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getPositionCovariance() const
{
  return P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getVelocityCovariance() const
{
  return P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getRotationCovariance() const
{
  return P_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getAccelBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaAccBiasIdx, kDeltaAccBiasIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getGyroBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaGyroBiasIdx, kDeltaGyroBiasIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getMagHardBiasCovariance() const
{
  return P_.block<3, 3>(kDeltaMagHardBiasIdx, kDeltaMagHardBiasIdx);
}

inline Eigen::Matrix6d ErrorStateKalmanFilter::getMagSoftBiasCovariance() const
{
  return P_.block<6, 6>(kDeltaMagSoftBiasIdx, kDeltaMagSoftBiasIdx);
}

inline double ErrorStateKalmanFilter::getGravityVariance() const
{
  return P_(kDeltaGravIdx, kDeltaGravIdx);
}

inline double ErrorStateKalmanFilter::getBaroAltBiasVariance() const
{
  return P_(kDeltaBaroAltBiasIdx, kDeltaBaroAltBiasIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition(const StateVector& x) const
{
  return x.segment<3>(kPosIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity(const StateVector& x) const
{
  return x.segment<3>(kVelIdx);
}

inline Eigen::Vector4d ErrorStateKalmanFilter::getHamilton(const StateVector& x) const
{
  return x.segment<4>(kQuatIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getAccelBias(const StateVector& x) const
{
  return x.segment<3>(kAccBiasIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGyroBias(const StateVector& x) const
{
  return x.segment<3>(kGyroBiasIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getMagHardBias(const StateVector& x) const
{
  return x.segment<3>(kMagHardBiasIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getMagSoftBias(const StateVector& x) const
{
  const Eigen::Vector6d tp = x.segment<6>(kMagSoftBiasIdx);
  const auto& a = tp(0);
  const auto& b = tp(1);
  const auto& c = tp(2);
  const auto& d = tp(3);
  const auto& e = tp(4);
  const auto& f = tp(5);
  return (Eigen::Matrix3d() << a, b, c, b, d, e, c, e, f).finished();
}

inline double ErrorStateKalmanFilter::getBaroAltBias(const StateVector& x) const
{
  return x(kBaroAltBiasIdx);
}

inline double ErrorStateKalmanFilter::getGravity(const StateVector& x) const
{
  return x(kGravIdx);
}

inline Eigen::Vector2d ErrorStateKalmanFilter::getXY(const StateVector& x) const
{
  return x.block<2, 1>(kPosIdx, 0);
}

inline double ErrorStateKalmanFilter::getAltitude(const StateVector& x) const
{
  return x(kAltIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition(const StateVector& x, const Eigen::Vector3d& offset) const
{
  return getPosition(x) + getQuaternion(x) * offset;
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity(
  const StateVector& x,
  const Eigen::Vector3d& offset,
  const Eigen::Vector3d& gyro_meas) const
{
  return getVelocity(x) + getQuaternion(x) * (gyro_meas - getGyroBias(x)).cross(offset);
}

inline Eigen::Quaterniond ErrorStateKalmanFilter::getQuaternion(const StateVector& x) const
{
  return eigen::quaternionFromHamilton(getHamilton(x));
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getDCM(const StateVector& x) const
{
  return getQuaternion(x).toRotationMatrix();
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGravVector(const StateVector& x) const
{
  return Eigen::Vector3d(0, 0, -getGravity(x));
}

template <int M>
double ErrorStateKalmanFilter::correct(
  const Eigen::Matrix<double, M, 1>& delta_meas,
  const Eigen::Matrix<double, M, M>& meas_cov,
  const Eigen::Matrix<double, M, kDeltaStateSize>& H)
{
  assert(eigen::isSymmetricPositiveDefinite(meas_cov));

  // (274) Compute kalman gain
  const Eigen::Matrix<double, kDeltaStateSize, M> PHt = P_ * H.transpose();
  const Eigen::Matrix<double, M, M> Sigma_inv = (H * PHt + meas_cov).inverse();
  const Eigen::Matrix<double, kDeltaStateSize, M> K = PHt * Sigma_inv;

  // (275) Compute error state
  const DeltaStateVector delta_x = K * delta_meas;

  // (276) Update covariance matrix
  const auto I = DeltaStateVector::Ones().asDiagonal();
  const DeltaStateMatrix I_KH = I - K * H;
  if (enable_joseph_form_) {
    // Easier to keep symmetric positive definiteness.
    const auto P1 = I_KH * P_.selfadjointView<Eigen::Lower>() * I_KH.transpose();
    const auto P2 = K * meas_cov.template selfadjointView<Eigen::Lower>() * K.transpose();
    P_ = P1 + P2;
  }
  else {
    // Theoretically correct, but numerically unstable.
    P_ = I_KH * P_;
  }

  // (283) Update state
  const Eigen::Vector3d dtheta = delta_x.segment<3>(kDeltaThetaIdx);
  const Eigen::Quaterniond q_dtheta = eigen::quaternionFromAngleAxis(dtheta);
  x_.segment<3>(kPosIdx) += delta_x.segment<3>(kDeltaPosIdx);
  x_.segment<3>(kVelIdx) += delta_x.segment<3>(kDeltaVelIdx);
  x_.segment<4>(kQuatIdx) = eigen::hamiltonFromQuaternion(getQuaternion() * q_dtheta);
  x_.segment<3>(kAccBiasIdx) += delta_x.segment<3>(kDeltaAccBiasIdx);
  x_.segment<3>(kGyroBiasIdx) += delta_x.segment<3>(kDeltaGyroBiasIdx);
  x_.segment<3>(kMagHardBiasIdx) += delta_x.segment<3>(kDeltaMagHardBiasIdx);
  x_.segment<6>(kMagSoftBiasIdx) += delta_x.segment<6>(kDeltaMagSoftBiasIdx);
  x_(kBaroAltBiasIdx) += delta_x(kDeltaBaroAltBiasIdx);
  x_(kGravIdx) += delta_x(kDeltaGravIdx);

  // (286) Initialize ESKF (Optional)
  if (enable_cov_initialization_) {
    G_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = Eigen::Diagonal3d(1, 1, 1) - eigen::skew(0.5 * dtheta);
    P_ = G_ * P_.selfadjointView<Eigen::Lower>() * G_.transpose();  // TODO: Compute only the required parts.
  }

  // Apply constraints to avoid numerical errors
  applyConstraints();

  // Compute anomaly score
  const double anomaly_score = (delta_meas.transpose() * Sigma_inv * delta_meas)(0) / M;
  return anomaly_score;
}
}  // namespace eskf
}  // namespace tobas
