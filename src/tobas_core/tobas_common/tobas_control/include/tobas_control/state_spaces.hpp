// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/core.hpp>

#include "./util.hpp"

namespace tobas::ctrl
{
class LinearDynamics
{
public:
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;

  inline explicit LinearDynamics(Eigen::Index _x_size, Eigen::Index _u_size);
  inline explicit LinearDynamics(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b);
  explicit LinearDynamics();

  inline Eigen::Index stateSize() const;
  inline Eigen::Index inputSize() const;

  inline void resize(Eigen::Index _x_size, Eigen::Index _u_size);
  inline void setZero();

  /** Compute A x + B u. */
  inline Eigen::VectorXd dynamics(const Eigen::VectorXd& _x, const Eigen::VectorXd& _u) const;

  inline bool isSizeMatch() const;
  inline bool isFinite() const;
  inline bool isControllable() const;

  /** Create state equations for scaled states and inputs. */
  LinearDynamics scale(const Eigen::VectorXd& _x_scale, const Eigen::VectorXd& _u_scale) const;

  friend std::ostream& operator<<(std::ostream& _os, const LinearDynamics& _arg);
};

class LinearStateSpace
{
public:
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;

  inline explicit LinearStateSpace(Eigen::Index _x_size, Eigen::Index _u_size, Eigen::Index _y_size);
  inline explicit LinearStateSpace(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b, const Eigen::MatrixXd& _c);
  inline explicit LinearStateSpace(const LinearDynamics& _dyn, const Eigen::MatrixXd& _c);
  inline explicit LinearStateSpace();

  inline LinearDynamics getDynamics() const;

  /** Update `A` and `B`. */
  inline void updateDynamics(const LinearDynamics& _dyn);

  inline Eigen::Index stateSize() const;
  inline Eigen::Index inputSize() const;
  inline Eigen::Index outputSize() const;

  inline void resize(Eigen::Index _x_size, Eigen::Index _u_size, Eigen::Index _y_size);
  inline void setZero();

  inline bool isSizeMatch() const;
  inline bool isFinite() const;
  inline bool isControllable() const;
  inline bool isObservable() const;

  friend std::ostream& operator<<(std::ostream& _os, const LinearStateSpace& _arg);
};

inline LinearDynamics::LinearDynamics(Eigen::Index _x_size, Eigen::Index _u_size)
  : A(_x_size, _x_size), B(_x_size, _u_size)
{
}

inline LinearDynamics::LinearDynamics(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b) : A(_a), B(_b)
{
  assert(A.cols() == stateSize());
  assert(B.rows() == stateSize());
}

inline LinearDynamics::LinearDynamics()
{
}

inline Eigen::Index LinearDynamics::stateSize() const
{
  return A.rows();
}

inline Eigen::Index LinearDynamics::inputSize() const
{
  return B.cols();
}

inline void LinearDynamics::resize(Eigen::Index _x_size, Eigen::Index _u_size)
{
  A.conservativeResize(_x_size, _x_size);
  B.conservativeResize(_x_size, _u_size);
}

inline void LinearDynamics::setZero()
{
  A.setZero();
  B.setZero();
}

inline bool LinearDynamics::isSizeMatch() const
{
  return A.rows() == A.cols() && A.cols() == B.rows();
}

inline Eigen::VectorXd LinearDynamics::dynamics(const Eigen::VectorXd& _x, const Eigen::VectorXd& _u) const
{
  assert(_x.size() == stateSize());
  assert(_u.size() == inputSize());

  return A * _x + B * _u;
}

inline bool LinearDynamics::isFinite() const
{
  return eigen::isFinite(A) && eigen::isFinite(B);
}

inline bool LinearDynamics::isControllable() const
{
  return ctrl::isControllable(A, B);
}

inline LinearStateSpace::LinearStateSpace(Eigen::Index _x_size, Eigen::Index _u_size, Eigen::Index _y_size)
  : A(_x_size, _x_size), B(_x_size, _u_size), C(_y_size, _x_size)
{
}

inline LinearStateSpace::LinearStateSpace(const Eigen::MatrixXd& _a, const Eigen::MatrixXd& _b, const Eigen::MatrixXd& _c)
  : A(_a), B(_b), C(_c)
{
  assert(A.cols() == stateSize());
  assert(B.rows() == stateSize());
  assert(C.cols() == stateSize());
}

inline LinearStateSpace::LinearStateSpace(const LinearDynamics& _dyn, const Eigen::MatrixXd& _c)
  : LinearStateSpace(_dyn.A, _dyn.B, _c)
{
}

inline LinearStateSpace::LinearStateSpace()
{
}

inline LinearDynamics LinearStateSpace::getDynamics() const
{
  return LinearDynamics(A, B);
}

inline void LinearStateSpace::updateDynamics(const LinearDynamics& _dyn)
{
  A = _dyn.A;
  B = _dyn.B;

  assert(A.cols() == stateSize());
  assert(B.rows() == stateSize());
  assert(C.cols() == stateSize());
}

inline Eigen::Index LinearStateSpace::stateSize() const
{
  return A.rows();
}

inline Eigen::Index LinearStateSpace::inputSize() const
{
  return B.cols();
}

inline Eigen::Index LinearStateSpace::outputSize() const
{
  return C.rows();
}

inline void LinearStateSpace::resize(Eigen::Index _x_size, Eigen::Index _u_size, Eigen::Index _y_size)
{
  A.conservativeResize(_x_size, _x_size);
  B.conservativeResize(_x_size, _u_size);
  C.conservativeResize(_y_size, _x_size);
}

inline void LinearStateSpace::setZero()
{
  A.setZero();
  B.setZero();
  C.setZero();
}

inline bool LinearStateSpace::isSizeMatch() const
{
  return A.rows() == A.cols() && A.cols() == B.rows() && A.cols() == C.cols();
}

inline bool LinearStateSpace::isFinite() const
{
  return eigen::isFinite(A) && eigen::isFinite(B) && eigen::isFinite(C);
}

inline bool LinearStateSpace::isControllable() const
{
  return ctrl::isControllable(A, B);
}

inline bool LinearStateSpace::isObservable() const
{
  return ctrl::isObservable(A, C);
}
}  // namespace tobas::ctrl
