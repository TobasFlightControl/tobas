#pragma once

#include <dh_eigen_tools/core.hpp>

#include "./util.hpp"

namespace ctrl
{
class LinearDynamics
{
public:
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;

  inline explicit LinearDynamics(const size_t& x_size, const size_t& u_size);
  inline explicit LinearDynamics(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
  explicit LinearDynamics();

  inline size_t stateSize() const;
  inline size_t inputSize() const;

  inline void resize(const size_t& x_size, const size_t& u_size);
  inline void setZero();

  /* Compute A x + B u. */
  inline Eigen::VectorXd dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;

  inline bool isSizeMatch() const;
  inline bool isFinite() const;
  inline bool isControllable() const;

  /* スケーリングされた状態と入力に対する状態方程式を作成． */
  LinearDynamics scale(const Eigen::VectorXd& x_scale, const Eigen::VectorXd& u_scale) const;

  friend std::ostream& operator<<(std::ostream& os, const LinearDynamics& arg);
};

class LinearStateSpace
{
public:
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;

  inline explicit LinearStateSpace(
    const size_t& x_size,
    const size_t& u_size,
    const size_t& y_size);
  inline explicit LinearStateSpace(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C);
  inline explicit LinearStateSpace(const LinearDynamics& dyn, const Eigen::MatrixXd& C);
  inline explicit LinearStateSpace();

  inline LinearDynamics getDynamics() const;

  /* A,Bを更新する． */
  inline void updateDynamics(const LinearDynamics& dyn);

  inline size_t stateSize() const;
  inline size_t inputSize() const;
  inline size_t outputSize() const;

  inline void resize(const size_t& x_size, const size_t& u_size, const size_t& y_size);
  inline void setZero();

  inline bool isSizeMatch() const;
  inline bool isFinite() const;
  inline bool isControllable() const;
  inline bool isObservable() const;

  friend std::ostream& operator<<(std::ostream& os, const LinearStateSpace& arg);
};

inline LinearDynamics::LinearDynamics(const size_t& x_size, const size_t& u_size)
  : A(x_size, x_size), B(x_size, u_size)
{
}

inline LinearDynamics::LinearDynamics(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B)
  : A(A), B(B)
{
  assert(static_cast<size_t>(A.cols()) == stateSize());
  assert(static_cast<size_t>(B.rows()) == stateSize());
}

inline LinearDynamics::LinearDynamics()
{
}

inline size_t LinearDynamics::stateSize() const
{
  return A.rows();
}

inline size_t LinearDynamics::inputSize() const
{
  return B.cols();
}

inline void LinearDynamics::resize(const size_t& x_size, const size_t& u_size)
{
  eigen_tools::resizeIfNecessary(A, x_size, x_size);
  eigen_tools::resizeIfNecessary(B, x_size, u_size);
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

inline Eigen::VectorXd
LinearDynamics::dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const
{
  assert(static_cast<size_t>(x.size()) == stateSize());
  assert(static_cast<size_t>(u.size()) == inputSize());

  return A * x + B * u;
}

inline bool LinearDynamics::isFinite() const
{
  return eigen_tools::isFinite(A) && eigen_tools::isFinite(B);
}

inline bool LinearDynamics::isControllable() const
{
  return ctrl::isControllable(A, B);
}

inline LinearStateSpace::LinearStateSpace(
  const size_t& x_size,
  const size_t& u_size,
  const size_t& y_size)
  : A(x_size, x_size), B(x_size, u_size), C(y_size, x_size)
{
}

inline LinearStateSpace::LinearStateSpace(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& C)
  : A(A), B(B), C(C)
{
  assert(static_cast<size_t>(A.cols()) == stateSize());
  assert(static_cast<size_t>(B.rows()) == stateSize());
  assert(static_cast<size_t>(C.cols()) == stateSize());
}

inline LinearStateSpace::LinearStateSpace(const LinearDynamics& dyn, const Eigen::MatrixXd& C)
  : LinearStateSpace(dyn.A, dyn.B, C)
{
}

inline LinearStateSpace::LinearStateSpace()
{
}

inline LinearDynamics LinearStateSpace::getDynamics() const
{
  return LinearDynamics(A, B);
}

inline void LinearStateSpace::updateDynamics(const LinearDynamics& dyn)
{
  A = dyn.A;
  B = dyn.B;

  assert(static_cast<size_t>(A.cols()) == stateSize());
  assert(static_cast<size_t>(B.rows()) == stateSize());
  assert(static_cast<size_t>(C.cols()) == stateSize());
}

inline size_t LinearStateSpace::stateSize() const
{
  return A.rows();
}

inline size_t LinearStateSpace::inputSize() const
{
  return B.cols();
}

inline size_t LinearStateSpace::outputSize() const
{
  return C.rows();
}

inline void
LinearStateSpace::resize(const size_t& x_size, const size_t& u_size, const size_t& y_size)
{
  eigen_tools::resizeIfNecessary(A, x_size, x_size);
  eigen_tools::resizeIfNecessary(B, x_size, u_size);
  eigen_tools::resizeIfNecessary(C, y_size, x_size);
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
  return eigen_tools::isFinite(A) && eigen_tools::isFinite(B) && eigen_tools::isFinite(C);
}

inline bool LinearStateSpace::isControllable() const
{
  return ctrl::isControllable(A, B);
}

inline bool LinearStateSpace::isObservable() const
{
  return ctrl::isObservable(A, C);
}
}  // namespace ctrl
