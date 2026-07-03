// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./state_spaces.hpp"

namespace tobas
{
namespace ctrl
{
/**
 * @brief Linear Kalman filter.
 * cf. https://www.tdupress.jp/book/b349390.html
 */
class KalmanFilter
{
public:
  LinearStateSpace ss;  // x(k+1) = A x(k) + B u(k), y(k) = C x(k): discrete-time state equation
  Eigen::MatrixXd Bv;   // Process noise matrix
  Eigen::MatrixXd Q;    // Process noise covariance
  Eigen::MatrixXd R;    // Observation noise covariance
  Eigen::VectorXd y;    // Observation
  Eigen::VectorXd u;    // Control input, if any

  explicit KalmanFilter();
  explicit KalmanFilter(
    const Eigen::Index& x_size,
    const Eigen::Index& u_size,
    const Eigen::Index& y_size,
    const Eigen::Index& v_size);

  void
  resize(const Eigen::Index& x_size, const Eigen::Index& u_size, const Eigen::Index& y_size, const Eigen::Index& v_size);
  void setZero();
  void initialize(const Eigen::VectorXd& init_x, const Eigen::MatrixXd& init_P);
  void update();

  inline Eigen::Index stateSize() const;
  inline Eigen::Index inputSize() const;
  inline Eigen::Index outputSize() const;
  inline Eigen::Index systemNoiseSize() const;

  inline const Eigen::VectorXd& state() const;
  inline const Eigen::MatrixXd& covariance() const;

private:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;

  void verify() const;
};

/* Estimate a constant value that includes white noise. */
class IdentityKalmanFilter
{
public:
  Eigen::MatrixXd Q;  // Process noise covariance
  Eigen::MatrixXd R;  // Observation noise covariance
  Eigen::VectorXd y;  // Observation

  explicit IdentityKalmanFilter(const Eigen::Index& size = 0);

  void resize(const Eigen::Index& size);
  void setZero();
  void initialize(const Eigen::VectorXd& init_x, const Eigen::MatrixXd& init_P);
  void update();

  inline const Eigen::VectorXd& state() const;
  inline const Eigen::MatrixXd& covariance() const;

private:
  KalmanFilter kf_;
};

inline Eigen::Index KalmanFilter::stateSize() const
{
  return ss.stateSize();
}

inline Eigen::Index KalmanFilter::inputSize() const
{
  return ss.inputSize();
}

inline Eigen::Index KalmanFilter::outputSize() const
{
  return ss.outputSize();
}

inline Eigen::Index KalmanFilter::systemNoiseSize() const
{
  return Bv.cols();
}

inline const Eigen::VectorXd& KalmanFilter::state() const
{
  return x_;
}

inline const Eigen::MatrixXd& KalmanFilter::covariance() const
{
  return P_;
}

inline const Eigen::VectorXd& IdentityKalmanFilter::state() const
{
  return kf_.state();
}

inline const Eigen::MatrixXd& IdentityKalmanFilter::covariance() const
{
  return kf_.covariance();
}
}  // namespace ctrl
}  // namespace tobas
