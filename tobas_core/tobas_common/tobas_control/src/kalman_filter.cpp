// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control/kalman_filter.hpp"

#include <eigen3/Eigen/LU>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/linalg.hpp>

#include "tobas_control/util.hpp"

namespace tobas
{
namespace ctrl
{
KalmanFilter::KalmanFilter()
{
}

KalmanFilter::KalmanFilter(
  const Eigen::Index& x_size,
  const Eigen::Index& u_size,
  const Eigen::Index& y_size,
  const Eigen::Index& v_size)
{
  resize(x_size, u_size, y_size, v_size);
}

void KalmanFilter::resize(
  const Eigen::Index& x_size,
  const Eigen::Index& u_size,
  const Eigen::Index& y_size,
  const Eigen::Index& v_size)
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

void KalmanFilter::initialize(const Eigen::VectorXd& init_x, const Eigen::MatrixXd& init_P)
{
  assert(init_x.size() == init_P.rows());
  assert(eigen::isSymmetricSemiPositiveDefinite(init_P));

  x_ = init_x;
  P_ = init_P;
}

void KalmanFilter::update()
{
  verify();

  // Use the lower triangular parts for computational efficiency.
  const auto _P = P_.selfadjointView<Eigen::Lower>();
  const auto _Q = Q.selfadjointView<Eigen::Lower>();
  const auto _R = R.selfadjointView<Eigen::Lower>();

  // Prior prediction.
  const Eigen::VectorXd x_prev = ss.A * x_ + ss.B * u;
  const Eigen::MatrixXd P_prev = ss.A * _P * ss.A.transpose() + Bv * _Q * Bv.transpose();

  // Posterior estimate.
  const Eigen::MatrixXd PCt = P_prev.selfadjointView<Eigen::Lower>() * ss.C.transpose();
  const Eigen::MatrixXd G = PCt * (ss.C * PCt + R).inverse();
  const Eigen::MatrixXd I_GC = Eigen::MatrixXd::Identity(stateSize(), stateSize()) - G * ss.C;
  x_ = x_prev + G * (y - ss.C * x_prev);
  P_ = I_GC * P_prev.selfadjointView<Eigen::Lower>() * I_GC.transpose() + G * _R * G.transpose();  // Joseph form
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
  assert(eigen::isFinite(Bv));
  assert(eigen::isFinite(Q));
  assert(eigen::isFinite(R));
  assert(eigen::isFinite(y));
  assert(eigen::isFinite(u));
  assert(ctrl::isControllable(ss.A, Bv));  // FIXME: Stabilizability should actually be sufficient.
  assert(ctrl::isObservable(ss.A, ss.C));  // FIXME: Detectability should actually be sufficient.
  assert(eigen::isSymmetricSemiPositiveDefinite(Q));
  assert(eigen::isSymmetricPositiveDefinite(R));
}

IdentityKalmanFilter::IdentityKalmanFilter(const Eigen::Index& size)
{
  resize(size);
}

void IdentityKalmanFilter::resize(const Eigen::Index& size)
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

void IdentityKalmanFilter::initialize(const Eigen::VectorXd& init_x, const Eigen::MatrixXd& init_P)
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
}  // namespace tobas
