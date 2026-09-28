// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eigen_tools/ellipsoid.hpp"

#include <iostream>

#include <eigen3/Eigen/Eigenvalues>
#include <eigen3/Eigen/LU>

using namespace Eigen;

namespace tobas
{
namespace eigen
{
Ellipsoid::Ellipsoid()
{
  setIdentity();
}

Ellipsoid::Ellipsoid(const Vector3d& b, const Vector6d& t)
{
  setHardBias(b);
  setSoftBias(t);
}

bool Ellipsoid::initialize(const EllipsoidCoefficients& coefs)
{
  // Ellipsoid equation: `x^T A x + b^T x + c = 0`.
  Matrix3d A;
  A << coefs.a_xx, coefs.a_xy, coefs.a_zx, coefs.a_xy, coefs.a_yy, coefs.a_yz, coefs.a_zx, coefs.a_yz, coefs.a_zz;
  Vector3d b;
  b << coefs.b_x, coefs.b_y, coefs.b_z;

  // Diagonalize `A`.
  const SelfAdjointEigenSolver<Matrix3d> eigen_solver(A);
  const Vector3d Lam = eigen_solver.eigenvalues();
  const Matrix3d P = eigen_solver.eigenvectors();

  const Vector3d Lam_inv = Lam.cwiseInverse();
  const Matrix3d A_inv = P * Lam_inv.asDiagonal() * P.transpose();
  const Vector3d A_inv_b = A_inv * b;
  const auto W = 0.25 * b.dot(A_inv_b) - coefs.c;

  // Calculate radii along the principal axes.
  const Vector3d r2 = W * Lam_inv;
  if (!(r2.array() > 0.0).all()) {
    std::cerr << "The given equation does not define an ellipsoid." << std::endl;
    return false;
  }
  const Vector3d r = r2.cwiseSqrt();

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

const Vector3d& Ellipsoid::getHardBias() const
{
  return b_;
}

void Ellipsoid::setHardBias(const Vector3d& b)
{
  b_ = b;
}

Vector6d Ellipsoid::getSoftBias() const
{
  return (Vector6d() << T_(0, 0), T_(1, 1), T_(2, 2), T_(0, 1), T_(1, 2), T_(2, 0)).finished();
}

void Ellipsoid::setSoftBias(const Vector6d& t)
{
  const auto& txx = t(0);
  const auto& tyy = t(1);
  const auto& tzz = t(2);
  const auto& txy = t(3);
  const auto& tyz = t(4);
  const auto& tzx = t(5);
  T_ << txx, txy, tzx, txy, tyy, tyz, tzx, tyz, tzz;
  T_inv_ = T_.inverse();
}
}  // namespace eigen
}  // namespace tobas
