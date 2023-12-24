#pragma once

#include <dh_eigen_tools/core.hpp>

namespace ctrl
{
class LinearEquation
{
public:
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  inline explicit LinearEquation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b);
  inline explicit LinearEquation(const size_t& var_size, const size_t& eq_size);
  inline explicit LinearEquation();

  inline static LinearEquation Zero(const size_t& var_size, const size_t& eq_size);

  inline void resize(const size_t& var_size, const size_t& eq_size);
  inline void setZero();

  inline bool isFinite() const;

  /* 変数の次元． */
  inline size_t variableSize() const;

  /* (不)等式の次元． */
  inline size_t equationSize() const;

  /* スケーリングされた変数に対する行列方程式を作成． */
  LinearEquation scale(const Eigen::VectorXd& scale) const;

  /* 変化率についての方程式を離散化して変化量についての方程式に変換． */
  LinearEquation discretise(const double& dt) const;

  friend std::ostream& operator<<(std::ostream& os, const LinearEquation& arg);
};

inline LinearEquation::LinearEquation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b)
  : A(A), b(b)
{
  assert(A.rows() == b.rows());
}

inline LinearEquation::LinearEquation(const size_t& var_size, const size_t& eq_size)
  : A(eq_size, var_size), b(eq_size)
{
}

inline LinearEquation::LinearEquation()
{
}

inline LinearEquation LinearEquation::Zero(const size_t& var_size, const size_t& eq_size)
{
  LinearEquation res(var_size, eq_size);
  res.setZero();
  return res;
}

inline void LinearEquation::resize(const size_t& var_size, const size_t& eq_size)
{
  eigen_tools::resizeIfNecessary(A, eq_size, var_size);
  eigen_tools::resizeIfNecessary(b, eq_size);
}

inline void LinearEquation::setZero()
{
  A.setZero();
  b.setZero();
}

inline bool LinearEquation::isFinite() const
{
  return eigen_tools::isFinite(A) && eigen_tools::isFinite(b);
}

inline size_t LinearEquation::variableSize() const
{
  return A.cols();
}

inline size_t LinearEquation::equationSize() const
{
  return A.rows();
}
}  // namespace ctrl
