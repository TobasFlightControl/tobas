#include <dh_std_tools/math.hpp>
#include <dh_eigen_tools/linalg.hpp>
#include <dh_linear_control/util.hpp>
#include <dh_linear_control/dare.hpp>

#include "../include/state_estimation_cascade/cartesian_filter.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;
using namespace ctrl;

namespace state_estimation_cascade
{
CartesianFilter::CartesianFilter()
{
  // ダイナミクスの固定部分
  A_.setIdentity();
  B_.setZero();
  C_.setZero();
  C_.block(kPosIdx, kPosIdx, 3, 3).diagonal().fill(1.);
  C_.block(kVelIdx, kVelIdx, 3, 3).diagonal().fill(1.);
  C_.block(kAccIdx, kAccIdx, 3, 3).diagonal().fill(1.);

  P_.setZero();
  Q_.setZero();
}

void CartesianFilter::initialize(
  const Vector3d& init_pos,
  const Vector3d& init_vel,
  const Vector3d& init_acc,
  const Vector3d& init_grav,
  const Matrix3d& init_pos_cov,
  const Matrix3d& init_vel_cov,
  const Matrix3d& init_acc_cov,
  const Matrix3d& init_grav_cov,
  const double& grav_var)
{
  assert(eigen_tools::isSymmetric(init_pos_cov) && eigen_tools::isSemiPositive(init_pos_cov));
  assert(eigen_tools::isSymmetric(init_vel_cov) && eigen_tools::isSemiPositive(init_vel_cov));
  assert(eigen_tools::isSymmetric(init_acc_cov) && eigen_tools::isSemiPositive(init_acc_cov));
  assert(eigen_tools::isSymmetric(init_grav_cov) && eigen_tools::isSemiPositive(init_grav_cov));
  assert(grav_var > 0.);

  x_.block(kPosIdx, 0, 3, 1) = init_pos;
  x_.block(kVelIdx, 0, 3, 1) = init_vel;
  x_.block(kAccIdx, 0, 3, 1) = init_acc;
  x_.block(kGravIdx, 0, 3, 1) = init_grav;

  // DAREを用いて先に共分散行列の極限値を求めることもできるが，ここでは初期の共分散の成長を考慮する
  P_.block(kPosIdx, kPosIdx, 3, 3) = init_pos_cov;
  P_.block(kVelIdx, kVelIdx, 3, 3) = init_vel_cov;
  P_.block(kAccIdx, kAccIdx, 3, 3) = init_acc_cov;
  P_.block(kGravIdx, kGravIdx, 3, 3) = init_grav_cov;

  Q_.block(3, 3, 3, 3).diagonal().fill(grav_var);
}

void CartesianFilter::configure(const double& grav_var)
{
  Q_.block(3, 3, 3, 3).diagonal().fill(grav_var);
}

void CartesianFilter::predict(const Quaterniond& quat, const Matrix3d& init_acc_cov, double dt)
{
  assert(dt > 0.);  // バグ予防のため一応dt = 0を許容しないでおく
  assert(dt < kImuTimeGapThreshold);

  A_.block(kPosIdx, kVelIdx, 3, 3).diagonal().fill(dt);
  A_.block(kVelIdx, kAccIdx, 3, 3) = quat.toRotationMatrix() * dt;
  A_.block(kVelIdx, kGravIdx, 3, 3).diagonal().fill(dt);

  B_.block(kAccIdx, 0, 3, 3).diagonal().fill(dt);
  B_.block(kGravIdx, 3, 3, 3).diagonal().fill(dt);

  Q_.block(0, 0, 3, 3) = init_acc_cov;

  x_ = A_ * x_;
  P_ = A_ * P_ * A_.transpose() + B_ * Q_ * B_.transpose();

  // 共分散行列を無理やり対称化 (これが必須)
  eigen_tools::symmetrise(P_);
}

void CartesianFilter::measureXYZ(const Vector3d& p_m, const Matrix3d& cov)
{
  assert(eigen_tools::isPositive(cov));

  const Vector3d dpos = p_m - getXYZ();
  const Matrix<double, 3, kStateSize> C = C_.block(kPosIdx, 0, 3, kStateSize);
  correct<3>(dpos, cov, C);
}

void CartesianFilter::measureXY(const Vector2d& xy_m, const Matrix2d& cov)
{
  assert(eigen_tools::isPositive(cov));

  const Vector2d dxy = xy_m - getXY();
  const Matrix<double, 2, kStateSize> C = C_.block(kPosIdx, 0, 2, kStateSize);
  correct<2>(dxy, cov, C);
}

void CartesianFilter::measureAltitude(const double& z_m, const double& var)
{
  assert(var > 0.);

  const double dz = z_m - getAltitude();
  const Matrix<double, 1, kStateSize> C = C_.block(kAltIdx, 0, 1, kStateSize);
  correct<1>(Scalar(dz), Scalar(var), C);
}

void CartesianFilter::measureVelocity(const Vector3d& v_m, const Matrix3d& cov)
{
  assert(eigen_tools::isPositive(cov));

  const Vector3d dvel = v_m - getVelocity();
  const Matrix<double, 3, kStateSize> C = C_.block(kVelIdx, 0, 3, kStateSize);
  correct<3>(dvel, cov, C);
}

void CartesianFilter::measureAcceleration(const Vector3d& a_m, const Matrix3d& cov)
{
  assert(eigen_tools::isPositive(cov));

  const Vector3d dacc = a_m - getAcceleration();
  const Matrix<double, 3, kStateSize> C = C_.block(kAccIdx, 0, 3, kStateSize);
  correct<3>(dacc, cov, C);
}

Vector3d CartesianFilter::getXYZ() const
{
  return x_.block(kPosIdx, 0, 3, 1);
}

Vector2d CartesianFilter::getXY() const
{
  return x_.block(kPosIdx, 0, 2, 1);
}

double CartesianFilter::getAltitude() const
{
  return x_(kAltIdx);
}

Vector3d CartesianFilter::getVelocity() const
{
  return x_.block(kVelIdx, 0, 3, 1);
}

Vector3d CartesianFilter::getAcceleration() const
{
  return x_.block(kAccIdx, 0, 3, 1);
}

Vector3d CartesianFilter::getGravity() const
{
  return x_.block(kGravIdx, 0, 3, 1);
}

Eigen::Matrix3d CartesianFilter::getPositionCovariance() const
{
  return P_.block(kPosIdx, kPosIdx, 3, 3);
}

Eigen::Matrix3d CartesianFilter::getVelocityCovariance() const
{
  return P_.block(kVelIdx, kVelIdx, 3, 3);
}

Eigen::Matrix3d CartesianFilter::getAccelerationCovariance() const
{
  return P_.block(kAccIdx, kAccIdx, 3, 3);
}

Eigen::Matrix3d CartesianFilter::getGravityCovariance() const
{
  return P_.block(kGravIdx, kGravIdx, 3, 3);
}
}  // namespace state_estimation_cascade
