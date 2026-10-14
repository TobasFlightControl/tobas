// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eigen_tools/ellipsoid.hpp"

#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>

namespace tobas
{
namespace eigen
{
Ellipsoid::Ellipsoid()
{
  setIdentity();
}

Ellipsoid::Ellipsoid(const Eigen::Vector3d& _b, const Eigen::Vector6d& _t)
{
  setHardBias(_b);
  setSoftBias(_t);
}

bool Ellipsoid::initialize(const EllipsoidCoefficients& _coefs)
{
  // Ellipsoid equation: `x^T A x + b^T x + c = 0`.
  Eigen::Matrix3d A;
  A << _coefs.a_xx, _coefs.a_xy, _coefs.a_zx, _coefs.a_xy, _coefs.a_yy, _coefs.a_yz, _coefs.a_zx, _coefs.a_yz,
    _coefs.a_zz;
  Eigen::Vector3d b;
  b << _coefs.b_x, _coefs.b_y, _coefs.b_z;

  // Diagonalize `A`.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(A);
  const Eigen::Vector3d Lam = eigen_solver.eigenvalues();
  const Eigen::Matrix3d P = eigen_solver.eigenvectors();

  const Eigen::Vector3d Lam_inv = Lam.cwiseInverse();
  const Eigen::Matrix3d A_inv = P * Lam_inv.asDiagonal() * P.transpose();
  const Eigen::Vector3d A_inv_b = A_inv * b;
  const auto W = 0.25 * b.dot(A_inv_b) - _coefs.c;

  // Calculate radii along the principal axes.
  const Eigen::Vector3d r2 = W * Lam_inv;
  if (!(r2.array() > 0.0).all()) {
    return false;  // The given equation does not define an ellipsoid.
  }
  const Eigen::Vector3d r = r2.cwiseSqrt();

  // Calculate biases.
  b_ = -0.5 * A_inv_b;
  T_ = P * r.asDiagonal() * P.transpose();
  T_inv_ = P * r.cwiseInverse().asDiagonal() * P.transpose();

  return true;
}

void Ellipsoid::setIdentity()
{
  b_.setZero();
  T_.setIdentity();
  T_inv_.setIdentity();
}

const Eigen::Vector3d& Ellipsoid::getHardBias() const
{
  return b_;
}

void Ellipsoid::setHardBias(const Eigen::Vector3d& _b)
{
  b_ = _b;
}

Eigen::Vector6d Ellipsoid::getSoftBias() const
{
  return (Eigen::Vector6d() << T_(0, 0), T_(1, 1), T_(2, 2), T_(0, 1), T_(1, 2), T_(2, 0)).finished();
}

void Ellipsoid::setSoftBias(const Eigen::Vector6d& _t)
{
  const auto& txx = _t(0);
  const auto& tyy = _t(1);
  const auto& tzz = _t(2);
  const auto& txy = _t(3);
  const auto& tyz = _t(4);
  const auto& tzx = _t(5);
  T_ << txx, txy, tzx, txy, tyy, tyz, tzx, tyz, tzz;
  T_inv_ = T_.inverse();
}
}  // namespace eigen
}  // namespace tobas
