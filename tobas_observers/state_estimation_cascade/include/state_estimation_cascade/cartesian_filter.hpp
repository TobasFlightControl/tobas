#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "./constants.hpp"

namespace state_estimation_cascade
{
class CartesianFilter
{
  using StateMatrix = Eigen::Matrix<double, kStateSize, kStateSize>;
  using StateVector = Eigen::Matrix<double, kStateSize, 1>;

public:
  explicit CartesianFilter();

  void initialize(
    const Eigen::Vector3d& init_pos,
    const Eigen::Vector3d& init_vel,
    const Eigen::Vector3d& init_acc,
    const Eigen::Vector3d& init_grav,
    const Eigen::Matrix3d& init_pos_cov,
    const Eigen::Matrix3d& init_vel_cov,
    const Eigen::Matrix3d& init_acc_cov,
    const Eigen::Matrix3d& init_grav_cov);

  void configure(const double& grav_var);

  void predict(const Eigen::Quaterniond& quat, const Eigen::Matrix3d& acc_cov, const double& dt);

  /* 絶対位置の観測． */
  void measurePosition(const Eigen::Vector3d& p_m, const Eigen::Matrix3d& cov);

  /* 絶対平面位置の観測． */
  void measureXY(const Eigen::Vector2d& xy_m, const Eigen::Matrix2d& cov);

  /* 絶対高度の観測. */
  void measureAltitude(const double& z_m, const double& var);

  /* 絶対速度の観測． */
  void measureVelocity(const Eigen::Vector3d& v_m, const Eigen::Matrix3d& cov);

  /* 期待座標系における加速度センサの観測． */
  void measureAcceleration(const Eigen::Vector3d& a_m, const Eigen::Matrix3d& cov);

  Eigen::Vector3d getPosition() const;
  Eigen::Vector2d getXY() const;
  double getAltitude() const;
  Eigen::Vector3d getVelocity() const;
  Eigen::Vector3d getAcceleration() const;
  Eigen::Vector3d getGravity() const;

  Eigen::Matrix3d getPositionCovariance() const;
  Eigen::Matrix3d getVelocityCovariance() const;
  Eigen::Matrix3d getAccelerationCovariance() const;
  Eigen::Matrix3d getGravityCovariance() const;

private:
  StateVector x_;
  StateMatrix A_;
  Eigen::Matrix<double, kStateSize, kInputSize> B_;
  Eigen::Matrix<double, kOutputSize, kStateSize> C_;
  StateMatrix P_;
  Eigen::Matrix<double, kInputSize, kInputSize> Q_;

  template <size_t M>
  void correct(
    const Eigen::Matrix<double, M, 1>& dy,
    const Eigen::Matrix<double, M, M>& cov,
    const Eigen::Matrix<double, M, kStateSize>& C);
};

template <size_t M>
void CartesianFilter::correct(
  const Eigen::Matrix<double, M, 1>& dy,
  const Eigen::Matrix<double, M, M>& cov,
  const Eigen::Matrix<double, M, kStateSize>& C)
{
  // カルマンゲインを計算
  Eigen::Matrix<double, kStateSize, M> PCt = P_ * C.transpose();
  Eigen::Matrix<double, kStateSize, M> G = PCt * (C * PCt + cov).inverse();

  // 状態を修正
  StateVector dx = G * dy;
  x_ += dx;

  // 共分散行列を修正
  StateMatrix I = StateMatrix::Identity();
  StateMatrix I_GC = I - G * C;
  // P_ = I_GC * P_;  // 理論式 (数値的に不安定)
  P_ = I_GC * P_ * I_GC.transpose() + G * cov * G.transpose();  // ジョセフ形式
}
}  // namespace state_estimation_cascade
