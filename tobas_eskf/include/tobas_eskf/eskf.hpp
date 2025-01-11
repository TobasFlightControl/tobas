#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_eigen_tools/linalg.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "./constants.hpp"

namespace eskf
{
/**
 * @brief 誤差状態カルマンフィルタ．
 * https://www.flight.t.u-tokyo.ac.jp/?p=800
 *
 * @note IMUフレームで考える．
 */
class ErrorStateKalmanFilter
{
  using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using StateVector = Eigen::Vector<double, kStateSize>;
  using RowStateVector = Eigen::RowVector<double, kStateSize>;
  using DeltaStateMatrix = Eigen::Matrix<double, kDeltaStateSize, kDeltaStateSize>;
  using DeltaStateVector = Eigen::Vector<double, kDeltaStateSize>;
  using RowDeltaStateVector = Eigen::RowVector<double, kDeltaStateSize>;

  static constexpr double kMaxAccBias = 1.;    // [m/s^2]
  static constexpr double kMaxGyroBias = 0.1;  // [rad/s]
  static constexpr double kMinGravity = 9.75;  // [m/s^2]
  static constexpr double kMaxGravity = 9.85;  // [m/s^2]

public:
  explicit ErrorStateKalmanFilter();

  void initialize(
    const Eigen::Vector3d& init_pos,
    const Eigen::Vector3d& init_vel,
    const Eigen::Quaterniond& init_quat,
    const Eigen::Matrix3d& init_pos_cov,
    const Eigen::Matrix3d& init_vel_cov,
    const Eigen::Matrix3d& init_dtheta_cov,
    const Eigen::Matrix3d& init_acc_bias_cov,
    const Eigen::Matrix3d& init_gyro_bias_cov,
    const double& init_grav_var,
    const std::chrono::steady_clock::time_point& time);

  void enableJosephForm(bool enable);
  void enableCovInitialization(bool enable);
  bool setAccBiasProcNoiseVar(double value);
  bool setGyroBiasProcNoiseVar(double value);
  bool setGravProcNoiseVar(double value);

  inline Eigen::Vector3d getPosition() const;
  inline Eigen::Vector3d getVelocity() const;
  inline Eigen::Quaterniond getQuaternion() const;
  inline Eigen::Vector3d getAccelBias() const;
  inline Eigen::Vector3d getGyroBias() const;
  inline double getGravity() const;

  inline Eigen::Matrix3d getPositionCovariance() const;
  inline Eigen::Matrix3d getVelocityCovariance() const;
  inline Eigen::Matrix3d getOrientationCovariance() const;
  inline Eigen::Matrix3d getAccelBiasCovariance() const;
  inline Eigen::Matrix3d getGyroBiasCovariance() const;
  inline double getGravityVariance() const;

  inline void setPosition(const Eigen::Vector3d& pos);
  inline void setQuaternion(const Eigen::Quaterniond& quat);
  inline void setReferenceMagneticField(const Eigen::Vector3d& mag_ref);

  /**
   * @brief 加速度とジャイロから次の状態を予測し，姿勢を補正する．
   *
   * @param acc_meas [m/s^2] 加速度の観測値
   * @param gyro_meas [rad/s] ジャイロの観測値
   * @param acc_cov [m^2/s^4] 加速度の観測ノイズの共分散
   * @param gyro_cov [rad^2/s^2] ジャイロの観測ノイズの共分散
   * @param grav_cov [m^2/s^4] 重力加速度の観測ノイズの共分散
   * @param time [s] 現在時刻
   */
  void measureIMU(
    const Eigen::Vector3d& acc_meas,
    const Eigen::Vector3d& gyro_meas,
    const Eigen::Matrix3d& acc_cov,
    const Eigen::Matrix3d& gyro_cov,
    const Eigen::Matrix3d& grav_cov,
    const std::chrono::steady_clock::time_point& time);

  /**
   * @brief 位置の観測をノミナル状態に反映させる．
   *
   * @param pos_meas 世界座標系で表現された位置の観測値
   * @param pos_cov 位置の観測ノイズの共分散
   * @param offset IMUフレームで表現された，IMUフレームに対する観測フレームのオフセット
   *
   * @return Anormaly score
   */
  double measurePosition(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& offset,
    const std::chrono::steady_clock::time_point& time);
  double measureXY(
    const Eigen::Vector2d& xy_meas,
    const Eigen::Matrix2d& xy_cov,
    const std::chrono::steady_clock::time_point& time);
  double measureAltitude(const double& z_meas, const double& z_var, const std::chrono::steady_clock::time_point& time);
  /**
   * @brief 速度の観測をノミナル状態に反映させる．
   *
   * @param pos_meas 世界座標系で表現された速度の観測値
   * @param pos_cov 速度の観測ノイズの共分散
   * @param offset IMUフレームで表現された，IMUフレームに対する観測フレームのオフセット
   * @param gyro_meas ジャイロセンサの読み
   *
   * @return Anormaly score
   */
  double measureVelocity(
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas,
    const std::chrono::steady_clock::time_point& time);
  double measurePosVel(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas,
    const std::chrono::steady_clock::time_point& time);
  double measureQuaternion(
    const Eigen::Quaterniond& q_meas,
    const Eigen::Matrix3d& theta_cov,
    const std::chrono::steady_clock::time_point& time);
  double measureMagneticField(
    const Eigen::Vector3d& mag_meas,
    const Eigen::Matrix3d& mag_cov,
    const std::chrono::steady_clock::time_point& time);

private:
  // Configuration
  bool use_joseph_form_ = true;
  bool do_cov_initialization_ = false;
  double acc_bias_proc_noise_var_ = 0.;   // [m^2/s^4] 加速度バイアスのプロセスノイズの分散
  double gyro_bias_proc_noise_var_ = 0.;  // [rad^2/s^2] ジャイロバイアスのプロセスノイズの分散
  double grav_proc_noise_var_ = 0.;       // [m^2/s^4] 重力加速度のプロセスノイズの分散

  StateVector x_;         // State vector of the filter
  DeltaStateMatrix P_;    // Covariance of the error state
  DeltaStateMatrix F_x_;  // Jacobian of the state transition
  DeltaStateMatrix G_;    // Jacobian of the error initialization

  Eigen::Matrix<double, 3, kDeltaStateSize> H_pos_;
  Eigen::Matrix<double, 2, kDeltaStateSize> H_xy_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_z_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_vel_;
  Eigen::Matrix<double, 6, kDeltaStateSize> H_pv_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_theta_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_acc_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_mag_;

  std::chrono::steady_clock::time_point t_last_imu_;
  tobas_std::TimestampedBuffer<StateVector> x_history_;
  Eigen::Vector3d mag_ref_ = Eigen::Vector3d::UnitX();

  inline Eigen::Vector3d getPosition(const StateVector& x) const;
  inline Eigen::Vector3d getVelocity(const StateVector& x) const;
  inline Eigen::Quaterniond getQuaternion(const StateVector& x) const;
  inline Eigen::Vector3d getAccelBias(const StateVector& x) const;
  inline Eigen::Vector3d getGyroBias(const StateVector& x) const;
  inline double getGravity(const StateVector& x) const;

  inline Eigen::Vector2d getXY(const StateVector& x) const;
  inline double getAltitude(const StateVector& x) const;
  inline Eigen::Vector3d getPosition(const StateVector& x, const Eigen::Vector3d& offset) const;
  inline Eigen::Vector3d
  getVelocity(const StateVector& x, const Eigen::Vector3d& offset, const Eigen::Vector3d& gyro_meas) const;
  inline Eigen::Vector4d getHamilton(const StateVector& x) const;
  inline Eigen::Matrix3d getDCM(const StateVector& x) const;
  inline Eigen::Vector3d getGravVector(const StateVector& x) const;

  /* (281) */
  Eigen::Matrix<double, 4, 3> getQ_dtheta(const StateVector& x) const;

  /* ベクトルvのqによる回転をqで偏微分したもの．d(q * v * q') / d(q)． */
  Eigen::Matrix<double, 3, 4> quatRotationDerivative(const StateVector& x, const Eigen::Vector3d& a) const;

  /**
   * @brief 重力方向の観測．姿勢の修正に用いる．
   * https://www.dropbox.com/s/ijfnlkvcep1w0f2/%E5%A7%BF%E5%8B%A2%E6%8E%A8%E5%AE%9A%E3%81%AE%E5%9F%BA%E7%A4%8E.pdf
   *
   * @param acc_meas 加速度センサの読み．
   * @param grav_cov 観測による修正量を決めるパラメータ．
   * 数式的には共分散として扱うが，センサノイズに加えて推定姿勢の分散も影響するため一般に正しい値は分からないから調整すべき．
   *
   * @return Anormaly score
   */
  double measureGravity(
    const Eigen::Vector3d& acc_meas,
    const Eigen::Matrix3d& grav_cov,
    const std::chrono::steady_clock::time_point& time);

  /**
   * @brief 観測から状態と共分散の事後推定を求める
   *
   * @tparam M 観測の次元
   * @param delta_meas 観測とノミナル状態の誤差
   * @param meas_cov 観測ノイズの共分散
   * @param H 観測方程式
   *
   * @return Anormaly score
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

inline Eigen::Quaterniond ErrorStateKalmanFilter::getQuaternion() const
{
  return getQuaternion(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getAccelBias() const
{
  return getAccelBias(x_);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGyroBias() const
{
  return getGyroBias(x_);
}

inline double ErrorStateKalmanFilter::getGravity() const
{
  return getGravity(x_);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getPositionCovariance() const
{
  return P_.block<3, 3>(kDeltaPosIdx, kDeltaPosIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getVelocityCovariance() const
{
  return P_.block<3, 3>(kDeltaVelIdx, kDeltaVelIdx);
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getOrientationCovariance() const
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

inline double ErrorStateKalmanFilter::getGravityVariance() const
{
  return P_(kDeltaGravIdx, kDeltaGravIdx);
}

inline void ErrorStateKalmanFilter::setPosition(const Eigen::Vector3d& pos)
{
  x_.segment<3>(kPosIdx) = pos;
}

inline void ErrorStateKalmanFilter::setQuaternion(const Eigen::Quaterniond& quat)
{
  x_.segment<4>(kQuatIdx) = eigen::quaternionToHamilton(quat).normalized();
}

inline void ErrorStateKalmanFilter::setReferenceMagneticField(const Eigen::Vector3d& mag_ref)
{
  mag_ref_ = mag_ref;
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition(const StateVector& x) const
{
  return x.segment<3>(kPosIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity(const StateVector& x) const
{
  return x.segment<3>(kVelIdx);
}

inline Eigen::Quaterniond ErrorStateKalmanFilter::getQuaternion(const StateVector& x) const
{
  return eigen::hamiltonToQuaternion(getHamilton(x));
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getAccelBias(const StateVector& x) const
{
  return x.segment<3>(kAccBiasIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGyroBias(const StateVector& x) const
{
  return x.segment<3>(kGyroBiasIdx);
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

inline Eigen::Vector4d ErrorStateKalmanFilter::getHamilton(const StateVector& x) const
{
  return x.segment<4>(kQuatIdx);
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
  const DeltaStateMatrix I_KH = DeltaStateMatrix::Identity() - K * H;
  if (use_joseph_form_)
    P_ = I_KH * P_ * I_KH.transpose() + K * meas_cov * K.transpose();  // 対称正定が保持されやすい
  else
    P_ = I_KH * P_;  // 理論通りだが数値的に不安定
  eigen::symmetrise(P_);

  // (283) Update state
  const Eigen::Vector3d dtheta = delta_x.segment<3>(kDeltaThetaIdx);
  const Eigen::Quaterniond q_dtheta = eigen::angleAxisToQuaternion(dtheta);
  x_.segment<3>(kPosIdx) += delta_x.segment<3>(kDeltaPosIdx);
  x_.segment<3>(kVelIdx) += delta_x.segment<3>(kDeltaVelIdx);
  x_.segment<4>(kQuatIdx) = eigen::quaternionToHamilton(getQuaternion() * q_dtheta).normalized();
  x_.segment<3>(kAccBiasIdx) += delta_x.segment<3>(kDeltaAccBiasIdx);
  x_.segment<3>(kGyroBiasIdx) += delta_x.segment<3>(kDeltaGyroBiasIdx);
  x_(kGravIdx) += delta_x(kDeltaGravIdx);

  // Clamp state
  // 事前知識を用いて最低限ありえない値にはならないようにする
  x_.segment<3>(kAccBiasIdx) = x_.segment<3>(kAccBiasIdx).cwiseMax(-kMaxAccBias).cwiseMin(kMaxAccBias);
  x_.segment<3>(kGyroBiasIdx) = x_.segment<3>(kGyroBiasIdx).cwiseMax(-kMaxGyroBias).cwiseMin(kMaxGyroBias);
  x_(kGravIdx) = std::clamp(x_(kGravIdx), kMinGravity, kMaxGravity);

  // (286) Initialize ESKF (Optional)
  if (do_cov_initialization_)
  {
    G_.block<3, 3>(kDeltaThetaIdx, kDeltaThetaIdx) = Eigen::Diagonal3d(1, 1, 1) - eigen::skew(0.5 * dtheta);
    P_ = G_ * P_ * G_.transpose();  // TODO: sympyを用いるなどして必要な部分のみ計算
    eigen::symmetrise(P_);
  }

  // Compute anormaly score
  const double anormaly_score = (delta_meas.transpose() * Sigma_inv * delta_meas)(0) / M;
  return anormaly_score;
}
}  // namespace eskf
