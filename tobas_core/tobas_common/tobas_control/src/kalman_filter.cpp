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

KalmanFilter::KalmanFilter(Eigen::Index _x_size, Eigen::Index _u_size, Eigen::Index _y_size, Eigen::Index _v_size)
{
  resize(_x_size, _u_size, _y_size, _v_size);
}

void KalmanFilter::resize(Eigen::Index _x_size, Eigen::Index _u_size, Eigen::Index _y_size, Eigen::Index _v_size)
{
  ss.resize(_x_size, _u_size, _y_size);
  Bv.conservativeResize(_x_size, _v_size);
  Q.conservativeResize(_v_size, _v_size);
  R.conservativeResize(_y_size, _y_size);
  y.conservativeResize(_y_size);
  u.conservativeResize(_u_size);
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

void KalmanFilter::initialize(const Eigen::VectorXd& _init_x, const Eigen::MatrixXd& _init_p)
{
  assert(_init_x.size() == _init_p.rows());
  assert(eigen::isSymmetricSemiPositiveDefinite(_init_p));

  x_ = _init_x;
  P_ = _init_p;
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

IdentityKalmanFilter::IdentityKalmanFilter(Eigen::Index _size)
{
  resize(_size);
}

void IdentityKalmanFilter::resize(Eigen::Index _size)
{
  kf_.resize(_size, 0, _size, _size);
  kf_.ss.A.setIdentity(_size, _size);
  kf_.ss.C.setIdentity(_size, _size);
  kf_.Bv.setIdentity(_size, _size);

  Q.conservativeResize(_size, _size);
  R.conservativeResize(_size, _size);
  y.conservativeResize(_size);
}

void IdentityKalmanFilter::setZero()
{
  Q.setZero();
  R.setZero();
  y.setZero();
}

void IdentityKalmanFilter::initialize(const Eigen::VectorXd& _init_x, const Eigen::MatrixXd& _init_p)
{
  kf_.initialize(_init_x, _init_p);
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
