#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tobas_eigen_tools/typedef.hpp>
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
  using StateVector = Eigen::Matrix<double, kStateSize, 1>;
  using RowStateVector = Eigen::Matrix<double, 1, kStateSize>;
  using DeltaStateMatrix = Eigen::Matrix<double, kDeltaStateSize, kDeltaStateSize>;
  using DeltaStateVector = Eigen::Matrix<double, kDeltaStateSize, 1>;
  using RowDeltaStateVector = Eigen::Matrix<double, 1, kDeltaStateSize>;

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
    const double& init_grav_var);

  inline Eigen::Vector3d getPosition() const;
  inline Eigen::Vector3d getPosition(const Eigen::Vector3d& offset) const;
  inline Eigen::Vector2d getXY() const;
  inline double getAltitude() const;
  inline Eigen::Vector3d getVelocity() const;
  inline Eigen::Vector3d getVelocity(const Eigen::Vector3d& offset, const Eigen::Vector3d& gyro_meas) const;
  inline Eigen::Quaterniond getQuaternion() const;
  inline Eigen::Vector3d getAccelBias() const;
  inline Eigen::Vector3d getGyroBias() const;
  inline double getGravity() const;
  inline Eigen::Vector3d getGravVector() const;
  inline Eigen::Matrix3d getDCM() const;
  inline double getYaw() const;

  inline Eigen::Matrix3d getPositionCovariance() const;
  inline Eigen::Matrix3d getVelocityCovariance() const;
  inline Eigen::Matrix3d getOrientationCovariance() const;
  inline Eigen::Matrix3d getAccelBiasCovariance() const;
  inline Eigen::Matrix3d getGyroBiasCovariance() const;
  inline double getGravityVariance() const;

  inline void setPosition(const Eigen::Vector3d& pos);
  inline void setQuaternion(const Eigen::Quaterniond& quat);

  /**
   * @brief 加速度とジャイロから次の状態を予測する．
   *
   * @param acc_meas [m/s^2] 加速度の観測値
   * @param gyro_meas [rad/s] ジャイロの観測値
   * @param acc_noise_var [m^2/s^4] 加速度の観測ノイズの分散
   * @param gyro_noise_var [rad^2/s^2] ジャイロの観測ノイズの分散
   * @param acc_bias_noise_var [m^2/s^4] 加速度バイアスの観測ノイズの分散
   * @param gyro_bias_noise_var [rad^2/s^2] ジャイロバイアスの観測ノイズの分散
   * @param dt [s] 前回の予測からの経過時間
   */
  void predictIMU(
    const Eigen::Vector3d& acc_meas,
    const Eigen::Vector3d& gyro_meas,
    const double& acc_noise_var,
    const double& gyro_noise_var,
    const double& acc_bias_noise_var,
    const double& gyro_bias_noise_var,
    const double& grav_var,
    const double& dt);

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
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());
  double measureXY(const Eigen::Vector2d& xy_meas, const Eigen::Matrix2d& xy_cov);
  double measureAltitude(const double& z_meas, const double& z_var);
  double measureVelocity(const Eigen::Vector3d& vel_meas, const Eigen::Matrix3d& vel_cov);
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
    const Eigen::Vector3d& gyro_meas);
  double measurePosVel(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas);
  double measureQuaternion(const Eigen::Quaterniond& q_meas, const Eigen::Matrix3d& theta_cov);
  double measureYaw(const double& yaw_meas, const double& yaw_var);

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
  double measureGravity(const Eigen::Vector3d& acc_meas, const Eigen::Matrix3d& grav_cov);

private:
  StateVector x_;         // State vector of the filter
  DeltaStateMatrix P_;    // Covariance of the error state
  DeltaStateMatrix F_x_;  // Jacobian of the state transition

  Eigen::Matrix<double, 3, kDeltaStateSize> H_pos_;
  Eigen::Matrix<double, 2, kDeltaStateSize> H_xy_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_z_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_vel_;
  Eigen::Matrix<double, 6, kDeltaStateSize> H_pv_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_theta_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_acc_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_mag_;

  /**
   * @brief クオータニオンをベクトルの形で得る．
   * (w,x,y,z)の順(ハミルトン)だから，w()などでアクセスするとずれることに注意！
   *
   * @return ハミルトン形式のクオータニオン
   */
  inline Eigen::Vector4d getHamilton() const;

  /* (281) */
  Eigen::Matrix<double, 4, 3> getQ_dtheta() const;

  /* ベクトルvのqによる回転をqで偏微分したもの．d(q * v * q') / d(q)． */
  Eigen::Matrix<double, 3, 4> quatRotationDerivative(const Eigen::Vector3d& a) const;

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

  void injectErrorState(const DeltaStateVector& error_state);
};

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition() const
{
  return x_.segment<3>(kPosIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition(const Eigen::Vector3d& offset) const
{
  return getPosition() + getQuaternion() * offset;
}

inline Eigen::Vector2d ErrorStateKalmanFilter::getXY() const
{
  return x_.block<2, 1>(kPosIdx, 0);
}

inline double ErrorStateKalmanFilter::getAltitude() const
{
  return x_(kAltIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity() const
{
  return x_.segment<3>(kVelIdx);
}

inline Eigen::Vector3d
ErrorStateKalmanFilter::getVelocity(const Eigen::Vector3d& offset, const Eigen::Vector3d& gyro_meas) const
{
  return getVelocity() + getQuaternion() * (gyro_meas - getGyroBias()).cross(offset);
}

inline Eigen::Quaterniond ErrorStateKalmanFilter::getQuaternion() const
{
  return eigen_tools::hamiltonToQuaternion(getHamilton());
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getAccelBias() const
{
  return x_.segment<3>(kAccBiasIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGyroBias() const
{
  return x_.segment<3>(kGyroBiasIdx);
}

inline double ErrorStateKalmanFilter::getGravity() const
{
  return x_(kGravIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGravVector() const
{
  return Eigen::Vector3d(0, 0, -getGravity());
}

inline Eigen::Matrix3d ErrorStateKalmanFilter::getDCM() const
{
  return getQuaternion().toRotationMatrix();
}

inline double ErrorStateKalmanFilter::getYaw() const
{
  const auto R_W_B = getDCM();
  return atan2(R_W_B(1, 0), R_W_B(0, 0));
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
  x_.segment<4>(kQuatIdx) = eigen_tools::quaternionToHamilton(quat).normalized();
}

inline Eigen::Vector4d ErrorStateKalmanFilter::getHamilton() const
{
  return x_.segment<4>(kQuatIdx);
}

template <int M>
double ErrorStateKalmanFilter::correct(
  const Eigen::Matrix<double, M, 1>& delta_meas,
  const Eigen::Matrix<double, M, M>& meas_cov,
  const Eigen::Matrix<double, M, kDeltaStateSize>& H)
{
  assert(eigen_tools::isSymmetricPositiveDefinite(meas_cov));

  // Kalman gain
  const Eigen::Matrix<double, kDeltaStateSize, M> PHt = P_ * H.transpose();
  const Eigen::Matrix<double, M, M> Sigma_inv = (H * PHt + meas_cov).inverse();
  const Eigen::Matrix<double, kDeltaStateSize, M> K = PHt * Sigma_inv;

  // Correct nominal state
  const DeltaStateVector error_state = K * delta_meas;
  injectErrorState(error_state);

  // Update covariance matrix
  const DeltaStateMatrix I_KH = DeltaStateMatrix::Identity() - K * H;
  // P_ = I_KH * P_;  // Simple form
  P_ = I_KH * P_ * I_KH.transpose() + K * meas_cov * K.transpose();  // Joseph form
  eigen_tools::symmetrise(P_);                                       // 対称化

  // Anormaly score
  const double anormaly_score = (delta_meas.transpose() * Sigma_inv * delta_meas)(0) / M;
  return anormaly_score;
}
}  // namespace eskf
