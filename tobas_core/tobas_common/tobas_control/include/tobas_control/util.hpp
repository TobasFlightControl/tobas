// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

#include "./equations.hpp"

namespace tobas
{
namespace ctrl
{
/**
 * Create the controllability matrix.
 *
 * @param _a,B Continuous-time dynamics.
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd ctrb(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b);

/**
 * Create the observability matrix.
 *
 * @param _a,C Continuous-time dynamics.
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd obsv(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _c);

/**
 * Determine whether the system is controllable.
 *
 * @param _a,B Continuous-time dynamics.
 * @return bool
 */
bool isControllable(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b);

/**
 * Determine whether the system is observable.
 *
 * @param _a,C Continuous-time dynamics.
 * @return bool
 */
bool isObservable(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _c);

/**
 * Create an equivalent matrix inequality (`A @ x <= b`) from the variable vector range (`lb <= x <= ub`).
 *
 * @param _lb Lower bound.
 * @param _ub Upper bound.
 * @param _inf Values greater than this are omitted from the matrix inequality.
 * @return `(A, b)` for `LinearEquation` `A @ x <= b`.
 */
LinearEquation matIneqFromRange(const Eigen::VectorXd& _lb, const Eigen::VectorXd& _ub, double _inf = 1e+12);

/**
 * Compute the position at time `t` when the position tracking error decays exponentially.
 *
 * @param _x0 Initial position.
 * @param _xd Target position.
 * @param _tau Decay time constant.
 * @param _t Time, elapsed from the initial time 0.
 * @return double Position at time `t`.
 */
double firstOrderPos(double _x0, double _xd, double _tau, double _t);

/**
 * Compute the position at time `t` when the velocity tracking error decays exponentially. (memo: 1-47)
 *
 * @param _x0 Initial position.
 * @param _v0 Initial velocity.
 * @param _vd Target velocity.
 * @param _tau Decay time constant.
 * @param _t Time, elapsed from the initial time 0.
 * @return double Position at time `t`.
 */
double firstOrderVel(double _x0, double _v0, double _vd, double _tau, double _t);
}  // namespace ctrl
}  // namespace tobas
