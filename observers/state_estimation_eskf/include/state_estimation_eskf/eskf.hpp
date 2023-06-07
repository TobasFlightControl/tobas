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
 */
class ErrorStateKalmanFilter
{
  using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using StateVector = Eigen::Matrix<double, kStateSize, 1>;
  using dStateMatrix = Eigen::Matrix<double, kDeltaStateSize, kDeltaStateSize>;
  using dStateVector = Eigen::Matrix<double, kDeltaStateSize, 1>;
  using Scalar = Eigen::Matrix<double, 1, 1>;

public:
  explicit ErrorStateKalmanFilter();

  void initialize(
    double acc_noise_density,
    double gyro_noise_density,
    double acc_random_walk,
    double gyro_random_walk,
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

  void predictIMU(const Eigen::Vector3d& a_m, const Eigen::Vector3d& w_m, double dt);

  void measureXYZ(const Eigen::Vector3d& pos_meas, const Eigen::Matrix3d& pos_cov);
  void measureXY(const Eigen::Vector2d& xy_meas, const Eigen::Matrix2d& xy_cov);
  void measureAltitude(const double& z_meas, const double& z_var);
  void measureVelocity(const Eigen::Vector3d& vel_meas, const Eigen::Matrix3d& vel_cov);
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
   */
  void measureMagneticField(const Eigen::Vector3d& mag_meas, const Eigen::Matrix3d& mag_cov);

private:
  double acc_noise_density_;   // [m/s^2/sqrt(Hz)]
  double gyro_noise_density_;  // [rad/s/sqrt(Hz)]
  double acc_random_walk_;     // [m/s^3/sqrt(Hz)]
  double gyro_random_walk_;    // [rad/s^2/sqrt(Hz)]

  Eigen::Vector3d grav_W_;     // Acceleration due to gravity wrt. world frame [m/s^2]
  Eigen::Vector3d mag_W_;      // Magnetic field wrt. world frame [T]

  StateVector nominal_state_;  // State vector of the filter
  dStateMatrix P_;             // Covariance of the error state
  dStateMatrix F_x_;           // Jacobian of the state transition

  /* (281) */
  Eigen::Matrix<double, 4, 3> getQ_dtheta();

  template <size_t M>
  void correct(
    const Eigen::Matrix<double, M, 1>& delta_meas,
    const Eigen::Matrix<double, M, M>& meas_cov,
    const Eigen::Matrix<double, M, kDeltaStateSize>& H);

  void injectErrorState(const dStateVector& error_state);

  /* クオータニオンをベクトルの形で得る．(w,x,y,z)の順(ハミルトン)であることに注意！ */
  inline Eigen::Vector4d getQuatVector() const
  {
    return nominal_state_.block<4, 1>(kQuatIdx, 0);
  }
};

template <size_t M>
void ErrorStateKalmanFilter::correct(
  const Eigen::Matrix<double, M, 1>& delta_meas,
  const Eigen::Matrix<double, M, M>& meas_cov,
  const Eigen::Matrix<double, M, kDeltaStateSize>& H)
{
  assert(eigen_tools::isPositive(meas_cov));
  assert(H.norm() > 0.);  // Hが変更されないバグがあったため，Hに非ゼロの要素が含まれることを保証．

  // Kalman gain
  const auto PHt = P_ * H.transpose();
  const auto K = PHt * (H * PHt + meas_cov).inverse();

  // Correction error state
  const auto error_state = K * delta_meas;
  const auto I_KH = dStateMatrix::Identity() - K * H;

  // Update P (simple form)
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
