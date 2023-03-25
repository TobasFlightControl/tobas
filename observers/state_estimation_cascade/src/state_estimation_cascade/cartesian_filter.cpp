#include <iostream>

#include <dh_std_tools/math.hpp>
#include <dh_eigen_tools/linalg.hpp>
#include <dh_linear_control/util.hpp>
#include <dh_linear_control/dare.hpp>

#include "../../include/state_estimation_cascade/cartesian_filter.hpp"

using namespace std;
using namespace Eigen;
using namespace dh_std;
using namespace ctrl;

CartesianFilter::CartesianFilter()
{
}

void CartesianFilter::initialize(
  const Vector3d& init_pos,
  const Vector3d& init_vel,
  const Vector3d& init_acc,
  const Vector3d& init_grav,
  const Matrix3d& pos_cov,
  const Matrix3d& vel_cov,
  const Matrix3d& acc_cov,
  const int& grav_var_exp)
{
  assert(eigen_tools::isSymmetric(pos_cov) && eigen_tools::isPositive(pos_cov));
  assert(eigen_tools::isSymmetric(vel_cov) && eigen_tools::isPositive(vel_cov));
  assert(eigen_tools::isSymmetric(acc_cov) && eigen_tools::isPositive(acc_cov));

  x_.block(POS_IDX, 0, 3, 1) = init_pos;
  x_.block(VEL_IDX, 0, 3, 1) = init_vel;
  x_.block(ACC_IDX, 0, 3, 1) = init_acc;
  x_.block(GRAV_IDX, 0, 3, 1) = init_grav;

  constexpr double dt = 1e-2;  // 適当な離散時間

  A_.setIdentity();
  A_.block(POS_IDX, VEL_IDX, 3, 3).diagonal().fill(dt);
  A_.block(VEL_IDX, ACC_IDX, 3, 3).diagonal().fill(dt);
  A_.block(VEL_IDX, GRAV_IDX, 3, 3).diagonal().fill(dt);

  B_.setZero();
  B_.block(ACC_IDX, 0, 3, 3).diagonal().fill(dt);
  B_.block(GRAV_IDX, 3, 3, 3).diagonal().fill(dt);

  C_.setZero();
  C_.block(POS_IDX, POS_IDX, 3, 3).diagonal().fill(1.);
  C_.block(VEL_IDX, VEL_IDX, 3, 3).diagonal().fill(1.);
  C_.block(ACC_IDX, ACC_IDX, 3, 3).diagonal().fill(1.);

  // 可制御性と可観測性を保証 (実際は可安定性と可検出性で十分)
  assert(isControllable(A_, B_));
  assert(isObservable(A_, C_));

  Q_.setZero();
  Q_.block(0, 0, 3, 3) = acc_cov;
  // 重力ベクトルの外乱は非常に小さいはず
  // システムを駆動するために適当な微小値を入れておく
  double grav_var = pow(10, grav_var_exp);
  Q_.block(3, 3, 3, 3).diagonal().fill(grav_var);

  Matrix<double, OUT_SIZE, OUT_SIZE> R;
  R.setZero();
  R.block(POS_IDX, POS_IDX, 3, 3) = pos_cov;
  R.block(VEL_IDX, VEL_IDX, 3, 3) = vel_cov;
  R.block(ACC_IDX, ACC_IDX, 3, 3) = acc_cov;

  P_ = dare(A_.transpose(), C_.transpose(), B_ * Q_ * B_.transpose(), R, DareMethod::Joseph);
}

void CartesianFilter::reconfigure(const int& grav_var_exp)
{
  double grav_var = pow(10, grav_var_exp);
  Q_.block(3, 3, 3, 3).diagonal().fill(grav_var);
}

void CartesianFilter::predict(const Quaterniond& quat, const Matrix3d& acc_cov, double dt)
{
  assert(dt >= 0.);

  A_.block(POS_IDX, VEL_IDX, 3, 3).diagonal().fill(dt);
  A_.block(VEL_IDX, ACC_IDX, 3, 3) = quat.toRotationMatrix() * dt;
  A_.block(VEL_IDX, GRAV_IDX, 3, 3).diagonal().fill(dt);

  B_.block(ACC_IDX, 0, 3, 3).diagonal().fill(dt);
  B_.block(GRAV_IDX, 3, 3, 3).diagonal().fill(dt);

  Q_.block(0, 0, 3, 3) = acc_cov;

  // TODO: ESKFを参考に更新部分を効率化
  x_ = A_ * x_;
  P_ = A_ * P_ * A_.transpose() + B_ * Q_ * B_.transpose();
  eigen_tools::symmetrise(P_);
}

void CartesianFilter::measurePosition3D(const Vector3d& p_m, const Matrix3d& cov)
{
  assert(eigen_tools::isPositive(cov));

  Vector3d dpos = p_m - getPosition3D();
  Matrix<double, 3, STATE_SIZE> C = C_.block(POS_IDX, 0, 3, STATE_SIZE);
  correct<3>(dpos, cov, C);
}

void CartesianFilter::measurePosition2D(const Vector2d& xy_m, const Matrix2d& cov)
{
  assert(eigen_tools::isPositive(cov));

  Vector2d dxy = xy_m - getPosition2D();
  Matrix<double, 2, STATE_SIZE> C = C_.block(POS_IDX, 0, 2, STATE_SIZE);
  correct<2>(dxy, cov, C);
}

void CartesianFilter::measureAltitude(const double& z_m, const double& var)
{
  assert(var > 0.);

  double dz = z_m - getAltitude();
  Matrix<double, 1, STATE_SIZE> C = C_.block(ALT_IDX, 0, 1, STATE_SIZE);
  correct<1>(Scalar(dz), Scalar(var), C);
}

void CartesianFilter::measureVelocity(const Vector3d& v_m, const Matrix3d& cov)
{
  assert(eigen_tools::isPositive(cov));

  Vector3d dvel = v_m - getVelocity();
  Matrix<double, 3, STATE_SIZE> C = C_.block(VEL_IDX, 0, 3, STATE_SIZE);
  correct<3>(dvel, cov, C);
}

void CartesianFilter::measureAcceleration(const Vector3d& a_m, const Matrix3d& cov)
{
  assert(eigen_tools::isPositive(cov));

  Vector3d dacc = a_m - getAcceleration();
  Matrix<double, 3, STATE_SIZE> C = C_.block(ACC_IDX, 0, 3, STATE_SIZE);
  correct<3>(dacc, cov, C);
}

Vector3d CartesianFilter::getPosition3D() const
{
  return x_.block(POS_IDX, 0, 3, 1);
}

Vector2d CartesianFilter::getPosition2D() const
{
  return x_.block(POS_IDX, 0, 2, 1);
}

double CartesianFilter::getAltitude() const
{
  return x_(ALT_IDX);
}

Vector3d CartesianFilter::getVelocity() const
{
  return x_.block(VEL_IDX, 0, 3, 1);
}

Vector3d CartesianFilter::getAcceleration() const
{
  return x_.block(ACC_IDX, 0, 3, 1);
}

Vector3d CartesianFilter::getGravity() const
{
  return x_.block(GRAV_IDX, 0, 3, 1);
}
