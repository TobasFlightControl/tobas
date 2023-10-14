#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dh_eigen_tools/typedef.hpp>
#include <dh_eigen_tools/linalg.hpp>
#include <dh_eigen_tools/geometry.hpp>

#include "./constants.hpp"

namespace state_estimation_eskf
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
    const Eigen::Vector3d& grav_W,
    const Eigen::Vector3d& mag_W,
    const Eigen::Vector3d& init_pos,
    const Eigen::Vector3d& init_vel,
    const Eigen::Quaterniond& init_quat,
    const Eigen::Matrix3d& init_pos_cov,
    const Eigen::Matrix3d& init_vel_cov,
    const Eigen::Matrix3d& init_dtheta_cov,
    const Eigen::Matrix3d& init_acc_bias_cov,
    const Eigen::Matrix3d& init_gyro_bias_cov);

  inline Eigen::Vector3d getPosition() const;
  inline Eigen::Vector3d getPosition(const Eigen::Vector3d& offset) const;
  inline Eigen::Vector2d getXY() const;
  inline double getAltitude() const;
  inline Eigen::Vector3d getVelocity() const;
  inline Eigen::Vector3d
  getVelocity(const Eigen::Vector3d& offset, const Eigen::Vector3d& gyro_meas) const;
  inline Eigen::Quaterniond getQuaternion() const;
  inline Eigen::Vector3d getAccelBias() const;
  inline Eigen::Vector3d getGyroBias() const;
  inline Eigen::Matrix3d getDCM() const;
  inline double getYaw() const;

  inline Eigen::Matrix3d getPositionCovariance() const;
  inline Eigen::Matrix3d getVelocityCovariance() const;
  inline Eigen::Matrix3d getOrientationCovariance() const;
  inline Eigen::Matrix3d getAccelBiasCovariance() const;
  inline Eigen::Matrix3d getGyroBiasCovariance() const;

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
    const double& dt);

  /**
   * @brief 位置の観測をノミナル状態に反映させる．
   *
   * @param pos_meas 世界座標系で表現された位置の観測値
   * @param pos_cov 位置の観測ノイズの共分散
   * @param offset IMUフレームで表現された，IMUフレームに対する観測フレームのオフセット
   */
  void measurePosition(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& offset = Eigen::Vector3d::Zero());
  void measureXY(const Eigen::Vector2d& xy_meas, const Eigen::Matrix2d& xy_cov);
  void measureAltitude(const double& z_meas, const double& z_var);
  void measureVelocity(const Eigen::Vector3d& vel_meas, const Eigen::Matrix3d& vel_cov);
  /**
   * @brief 速度の観測をノミナル状態に反映させる．
   *
   * @param pos_meas 世界座標系で表現された速度の観測値
   * @param pos_cov 速度の観測ノイズの共分散
   * @param offset IMUフレームで表現された，IMUフレームに対する観測フレームのオフセット
   * @param gyro_meas ジャイロセンサの読み
   */
  void measureVelocity(
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas);
  void measurePosVel(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& gyro_meas);
  void measureQuaternion(const Eigen::Quaterniond& q_meas, const Eigen::Matrix3d& theta_cov);

  /**
   * @brief 重力方向の観測．姿勢の修正に用いる．
   * https://www.dropbox.com/s/ijfnlkvcep1w0f2/%E5%A7%BF%E5%8B%A2%E6%8E%A8%E5%AE%9A%E3%81%AE%E5%9F%BA%E7%A4%8E.pdf
   *
   * @param acc_meas 加速度センサの読み．
   * @param grav_cov 観測による修正量を決めるパラメータ．
   * 数式的には共分散として扱うが，センサノイズに加えて推定姿勢の分散も影響するため一般に正しい値は分からないから調整すべき．
   */
  void measureGravity(const Eigen::Vector3d& acc_meas, const Eigen::Matrix3d& grav_cov);

  /**
   * @brief 地磁気の観測．姿勢の修正に用いる．
   *
   * @param mag_meas 地磁気センサの読み．
   * @param mag_cov 観測による修正量を決めるパラメータ．
   * 数式的には共分散として扱うが，センサノイズに加えて推定姿勢の分散も影響するため一般に正しい値は分からないから調整すべき．
   *
   * @note
   * 地磁気センサのバイアスが大きく，ロールピッチの観測に用いると姿勢推定の精度が落ちる恐れがあるため，
   * 地磁気はヨー角の観測にのみ用いるべきという意見もある．
   */
  void measureMagneticField(const Eigen::Vector3d& mag_meas, const Eigen::Matrix3d& mag_cov);

  /**
   * @brief 地磁気の観測．ヨー角の修正に用いる．
   * https://github.com/PX4/PX4-ECL/blob/b3fed06fe822d08d19ab1d2c2f8daf7b7d21951c/EKF/mag_fusion.cpp#L420
   *
   * @param mag_meas 地磁気センサの読み．
   * @param yaw_var 観測による修正量を決めるパラメータ．
   * 数式的には共分散として扱うが，センサノイズに加えて推定姿勢の分散も影響するため一般に正しい値は分からないから調整すべき．
   */
  void
  measureMagneticField(const double& mag_meas_x, const double& mag_meas_y, const double& yaw_var);

private:
  bool is_initialized_ = false;

  Eigen::Vector3d grav_W_;  // Acceleration due to gravity wrt. world frame [m/s^2]
  Eigen::Vector3d mag_W_;   // Magnetic field wrt. world frame [T]

  StateVector nominal_state_;  // State vector of the filter
  DeltaStateMatrix P_;         // Covariance of the error state
  DeltaStateMatrix F_x_;       // Jacobian of the state transition

  Eigen::Matrix<double, 3, kDeltaStateSize> H_pos_;
  Eigen::Matrix<double, 2, kDeltaStateSize> H_xy_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_z_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_vel_;
  Eigen::Matrix<double, 6, kDeltaStateSize> H_pv_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_theta_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_acc_;
  Eigen::Matrix<double, 3, kDeltaStateSize> H_mag_rpy_;
  Eigen::Matrix<double, 1, kDeltaStateSize> H_mag_yaw_;

  /**
   * @brief クオータニオンをベクトルの形で得る．
   * (w,x,y,z)の順(ハミルトン)だから，w()などでアクセスするとずれることに注意！
   *
   * @return Eigen::Vector4d ハミルトン形式のクオータニオン
   */
  Eigen::Vector4d getHamilton() const;

  /* (281) */
  Eigen::Matrix<double, 4, 3> getQ_dtheta() const;

  /* ベクトルvのqによる回転をqで偏微分したもの．d(q * v * q') / d(q)． */
  Eigen::Matrix<double, 3, 4> quatRotationDerivative(const Eigen::Vector3d& a) const;

  template <size_t M>
  void correct(
    const Eigen::Matrix<double, M, 1>& delta_meas,
    const Eigen::Matrix<double, M, M>& meas_cov,
    const Eigen::Matrix<double, M, kDeltaStateSize>& H);

  void injectErrorState(const DeltaStateVector& error_state);
};

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition() const
{
  return nominal_state_.block<3, 1>(kPosIdx, 0);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getPosition(const Eigen::Vector3d& offset) const
{
  return getPosition() + getQuaternion() * offset;
}

inline Eigen::Vector2d ErrorStateKalmanFilter::getXY() const
{
  return nominal_state_.block<2, 1>(kPosIdx, 0);
}

inline double ErrorStateKalmanFilter::getAltitude() const
{
  return nominal_state_(kAltIdx);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity() const
{
  return nominal_state_.block<3, 1>(kVelIdx, 0);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getVelocity(
  const Eigen::Vector3d& offset,
  const Eigen::Vector3d& gyro_meas) const
{
  return getVelocity() + getQuaternion() * (gyro_meas - getGyroBias()).cross(offset);
}

inline Eigen::Quaterniond ErrorStateKalmanFilter::getQuaternion() const
{
  return eigen_tools::hamiltonToQuaternion(getHamilton());
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getAccelBias() const
{
  return nominal_state_.block<3, 1>(kAccBiasIdx, 0);
}

inline Eigen::Vector3d ErrorStateKalmanFilter::getGyroBias() const
{
  return nominal_state_.block<3, 1>(kGyroBiasIdx, 0);
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

template <size_t M>
void ErrorStateKalmanFilter::correct(
  const Eigen::Matrix<double, M, 1>& delta_meas,
  const Eigen::Matrix<double, M, M>& meas_cov,
  const Eigen::Matrix<double, M, kDeltaStateSize>& H)
{
  assert(eigen_tools::isSymmetricPositiveDefinite(meas_cov));

  // Kalman gain
  const Eigen::Matrix<double, kDeltaStateSize, M> PHt = P_ * H.transpose();
  const Eigen::Matrix<double, kDeltaStateSize, M> K = PHt * (H * PHt + meas_cov).inverse();

  // Correction error state
  const DeltaStateVector error_state = K * delta_meas;
  const DeltaStateMatrix I_KH = DeltaStateMatrix::Identity() - K * H;

  // Update covariance matrix
  // P_ = I_KH * P_;  // Simple form
  P_ = I_KH * P_ * I_KH.transpose() + K * meas_cov * K.transpose();  // Joseph form
  eigen_tools::symmetrise(P_);                                       // 対称化

  injectErrorState(error_state);

  // For debug
  // std::cout << "Delta measure:" << std::endl << delta_meas << std::endl;
  // std::cout << "Measurement covariance matrix:" << std::endl << meas_cov << std::endl;
  // std::cout << "Output matrix:" << std::endl << H << std::endl;
  // std::cout << "Kalman gain:" << std::endl << K << std::endl;
}
}  // namespace state_estimation_eskf
