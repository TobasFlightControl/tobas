#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <dh_eigen_tools/linalg.hpp>

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
  using Scalar = Eigen::Matrix<double, 1, 1>;

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

  Eigen::Vector3d getXYZ() const;
  Eigen::Vector2d getXY() const;
  double getAltitude() const;
  Eigen::Vector3d getVelocity() const;
  Eigen::Quaterniond getQuaternion() const;
  Eigen::Vector3d getAccelBias() const;
  Eigen::Vector3d getGyroBias() const;
  Eigen::Matrix3d getDCM() const;
  double getYaw() const;

  Eigen::Matrix3d getPositionCovariance() const;
  Eigen::Matrix3d getVelocityCovariance() const;
  Eigen::Matrix3d getOrientationCovariance() const;
  Eigen::Matrix3d getAccelBiasCovariance() const;
  Eigen::Matrix3d getGyroBiasCovariance() const;

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

  void measureXYZ(const Eigen::Vector3d& pos_meas, const Eigen::Matrix3d& pos_cov);
  /**
   * @brief 位置の観測をノミナル状態に反映させる．
   *
   * @param pos_meas 世界座標系で表現された位置の観測値
   * @param pos_cov 位置の観測ノイズの共分散
   * @param offset IMUフレームで表現された，IMUフレームに対する観測フレームのオフセット
   */
  void measureXYZ(
    const Eigen::Vector3d& pos_meas,
    const Eigen::Matrix3d& pos_cov,
    const Eigen::Vector3d& offset);
  void measureXY(const Eigen::Vector2d& xy_meas, const Eigen::Matrix2d& xy_cov);
  void measureAltitude(const double& z_meas, const double& z_var);
  void measureVelocity(const Eigen::Vector3d& vel_meas, const Eigen::Matrix3d& vel_cov);
  /**
   * @brief 速度の観測をノミナル状態に反映させる．
   *
   * @param pos_meas 世界座標系で表現された速度の観測値
   * @param pos_cov 速度の観測ノイズの共分散
   * @param gyro_meas ジャイロセンサの読み
   * @param offset IMUフレームで表現された，IMUフレームに対する観測フレームのオフセット
   */
  void measureVelocity(
    const Eigen::Vector3d& vel_meas,
    const Eigen::Matrix3d& vel_cov,
    const Eigen::Vector3d& gyro_meas,
    const Eigen::Vector3d& offset);
  void measureQuaternion(const Eigen::Quaterniond& q_meas, const Eigen::Matrix3d& theta_cov);

  /**
   * @brief 重力方向の観測．姿勢の修正に用いる．
   * https://www.dropbox.com/s/ijfnlkvcep1w0f2/%E5%A7%BF%E5%8B%A2%E6%8E%A8%E5%AE%9A%E3%81%AE%E5%9F%BA%E7%A4%8E.pdf
   *
   * @param acc_meas 加速度センサの読み．
   * @param cov 観測による修正量を決めるパラメータ．
   * 数式的には共分散として扱うが，センサノイズに加えて推定姿勢の分散も影響するため一般に正しい値は分からないから調整すべき．
   */
  void measureAcceleration(const Eigen::Vector3d& acc_meas, const Eigen::Matrix3d& acc_cov);

  /**
   * @brief 地磁気の観測．姿勢の修正に用いる．
   *
   * @param mag_meas 地磁気センサの読み．
   * @param cov 観測による修正量を決めるパラメータ．
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
  Eigen::Vector3d grav_W_;  // Acceleration due to gravity wrt. world frame [m/s^2]
  Eigen::Vector3d mag_W_;   // Magnetic field wrt. world frame [T]

  StateVector nominal_state_;  // State vector of the filter
  DeltaStateMatrix P_;         // Covariance of the error state
  DeltaStateMatrix F_x_;       // Jacobian of the state transition

  /**
   * @brief クオータニオンをベクトルの形で得る．
   * (w,x,y,z)の順(ハミルトン)だから，w()などでアクセスするとずれることに注意！
   *
   * @return Eigen::Vector4d ハミルトン形式のクオータニオン
   */
  Eigen::Vector4d getHamilton() const;

  /* (281) */
  Eigen::Matrix<double, 4, 3> getQ_dtheta() const;

  /* vのqによる回転をqで偏微分したもの．d(q * v * q') / d(q)． */
  Eigen::Matrix<double, 3, 4> quatRotationDerivative(const Eigen::Vector3d& a) const;

  template <size_t M>
  void correct(
    const Eigen::Matrix<double, M, 1>& delta_meas,
    const Eigen::Matrix<double, M, M>& meas_cov,
    const Eigen::Matrix<double, M, kDeltaStateSize>& H);

  void injectErrorState(const DeltaStateVector& error_state);
};

template <size_t M>
void ErrorStateKalmanFilter::correct(
  const Eigen::Matrix<double, M, 1>& delta_meas,
  const Eigen::Matrix<double, M, M>& meas_cov,
  const Eigen::Matrix<double, M, kDeltaStateSize>& H)
{
  assert(eigen_tools::isSymmetricPositiveDefinite(meas_cov));
  assert(H.norm() > 0.);  // Hが変更されないバグがあったため，Hに非ゼロの要素が含まれることを保証．

  // Kalman gain
  const auto PHt = P_ * H.transpose();
  const auto K = PHt * (H * PHt + meas_cov).inverse();

  // Correction error state
  const auto error_state = K * delta_meas;
  const auto I_KH = DeltaStateMatrix::Identity() - K * H;

  // Update covariance matrix
  // P_ = I_KH * P_;  // Simple form
  P_ = I_KH * P_ * I_KH.transpose() + K * meas_cov * K.transpose();  // Joseph form

  injectErrorState(error_state);

  // For debug
  // std::cout << "Delta measure:" << std::endl << delta_meas << std::endl;
  // std::cout << "Measurement covariance matrix:" << std::endl << meas_cov << std::endl;
  // std::cout << "Output matrix:" << std::endl << H << std::endl;
  // std::cout << "Kalman gain:" << std::endl << K << std::endl;
}
}  // namespace state_estimation_eskf
