// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <vector>

#include <eigen3/Eigen/Core>

#include "./equations.hpp"

namespace tobas
{
namespace ctrl
{
/**
 * @brief Create the controllability matrix.
 *
 * @param A,B Continuous-time dynamics.
 *
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd ctrb(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

/**
 * @brief Create the observability matrix.
 *
 * @param A,C Continuous-time dynamics.
 *
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd obsv(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C);

/**
 * @brief Determine whether the system is controllable.
 *
 * @param A,B Continuous-time dynamics.
 *
 * @return bool
 */
bool isControllable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);

/**
 * @brief Determine whether the system is observable.
 *
 * @param A,C Continuous-time dynamics.
 *
 * @return bool
 */
bool isObservable(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C);

/**
 * @brief Create an equivalent matrix inequality (`A @ x <= b`) from the variable vector range (`lb <= x <= ub`).
 *
 * @param lb Lower bound.
 * @param ub Upper bound.
 * @param inf Values greater than this are omitted from the matrix inequality.
 *
 * @return `(A, b)` for `LinearEquation` `A @ x <= b`.
 */
LinearEquation matIneqFromRange(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const double& inf = 1e+12);

/**
 * @brief Compute the position at time `t` when the position tracking error decays exponentially.
 *
 * @param x0 Initial position.
 * @param xd Target position.
 * @param tau Decay time constant.
 * @param t Time, elapsed from the initial time 0.
 *
 * @return double Position at time `t`.
 */
double firstOrderPos(const double& x0, const double& xd, const double& tau, const double& t);

/**
 * @brief Compute the position at time `t` when the velocity tracking error decays exponentially. (memo: 1-47)
 *
 * @param x0 Initial position.
 * @param v0 Initial velocity.
 * @param vd Target velocity.
 * @param tau Decay time constant.
 * @param t Time, elapsed from the initial time 0.
 *
 * @return double Position at time `t`.
 */
double firstOrderVel(const double& x0, const double& v0, const double& vd, const double& tau, const double& t);
}  // namespace ctrl
}  // namespace tobas
