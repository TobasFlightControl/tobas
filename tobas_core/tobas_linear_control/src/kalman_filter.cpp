#include <Eigen/LU>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/linalg.hpp>

#include "../include/tobas_linear_control/util.hpp"
#include "../include/tobas_linear_control/dare.hpp"
#include "../include/tobas_linear_control/kalman_filter.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
KalmanFilter::KalmanFilter()
{
}

KalmanFilter::KalmanFilter(const Index& x_size, const Index& u_size, const Index& y_size, const Index& v_size)
{
  resize(x_size, u_size, y_size, v_size);
}

void KalmanFilter::resize(const Index& x_size, const Index& u_size, const Index& y_size, const Index& v_size)
{
  ss.resize(x_size, u_size, y_size);
  Bv.conservativeResize(x_size, v_size);
  Q.conservativeResize(v_size, v_size);
  R.conservativeResize(y_size, y_size);
  y.conservativeResize(y_size);
  u.conservativeResize(u_size);
}

void KalmanFilter::setZero()
{
  ss.setZero();
  Bv.setZero();
  Q.setZero();
  R.setZero();
  y.setZero();
  u.setZero();
}

void KalmanFilter::initialize(const VectorXd& init_x, const MatrixXd& init_P)
{
  assert(init_x.size() == init_P.rows());
  assert(eigen_tools::isSymmetricSemiPositiveDefinite(init_P));

  x_ = init_x;
  P_ = init_P;
}

void KalmanFilter::update()
{
  verify();

  // 事前予測
  const VectorXd x_prev = ss.A * x_ + ss.B * u;
  const MatrixXd P_prev = ss.A * P_ * ss.A.transpose() + Bv * Q * Bv.transpose();

  // 事後推定
  const MatrixXd PCt = P_prev * ss.C.transpose();
  const MatrixXd G = PCt * (ss.C * PCt + R).inverse();
  const MatrixXd I_GC = MatrixXd::Identity(stateSize(), stateSize()) - G * ss.C;
  x_ = x_prev + G * (y - ss.C * x_prev);
  P_ = I_GC * P_prev * I_GC.transpose() + G * R * G.transpose();  // Joseph form

  // 強制対称化
  eigen_tools::symmetrise(P_);
}

void KalmanFilter::verify() const
{
  assert(ss.isSizeMatch());
  assert(Bv.rows() == stateSize() && Bv.cols() == systemNoiseSize());
  assert(Q.rows() == systemNoiseSize() && Q.cols() == systemNoiseSize());
  assert(R.rows() == outputSize() && R.cols() == outputSize());
  assert(y.size() == outputSize());
  assert(u.size() == inputSize());
  assert(ss.isFinite());
  assert(eigen_tools::isFinite(Bv));
  assert(eigen_tools::isFinite(Q));
  assert(eigen_tools::isFinite(R));
  assert(eigen_tools::isFinite(y));
  assert(eigen_tools::isFinite(u));
  assert(ctrl::isControllable(ss.A, Bv));  // FIXME: 本当は可安定で十分
  assert(ctrl::isObservable(ss.A, ss.C));  // FIXME: 本当は可検出で十分
  assert(eigen_tools::isSymmetricSemiPositiveDefinite(Q));
  assert(eigen_tools::isSymmetricPositiveDefinite(R));
}

IdentityKalmanFilter::IdentityKalmanFilter(const Index& size)
{
  resize(size);
}

void IdentityKalmanFilter::resize(const Index& size)
{
  kf_.resize(size, 0, size, size);
  kf_.ss.A.setIdentity(size, size);
  kf_.ss.C.setIdentity(size, size);
  kf_.Bv.setIdentity(size, size);

  Q.conservativeResize(size, size);
  R.conservativeResize(size, size);
  y.conservativeResize(size);
}

void IdentityKalmanFilter::setZero()
{
  Q.setZero();
  R.setZero();
  y.setZero();
}

void IdentityKalmanFilter::initialize(const VectorXd& init_x, const MatrixXd& init_P)
{
  kf_.initialize(init_x, init_P);
}

void IdentityKalmanFilter::update()
{
  kf_.Q = Q;
  kf_.R = R;
  kf_.y = y;

  kf_.update();
}
}  // namespace ctrl
