#pragma once

#include <tobas_eigen_tools/core.hpp>

namespace ctrl
{
class LinearEquation
{
public:
  Eigen::MatrixXd A;
  Eigen::VectorXd b;

  inline explicit LinearEquation(const Eigen::MatrixXd& A, const Eigen::MatrixXd& b);
  inline explicit LinearEquation(const Eigen::Index& var_size, const Eigen::Index& eq_size);
  inline explicit LinearEquation();

  inline static LinearEquation Zero(const Eigen::Index& var_size, const Eigen::Index& eq_size);

  inline void resize(const Eigen::Index& var_size, const Eigen::Index& eq_size);
  inline void setZero();

  inline bool isFinite() const;

  /* 変数の次元． */
  inline Eigen::Index variableSize() const;

  /* (不)等式の次元． */
  inline Eigen::Index equationSize() const;

  /* スケーリングされた変数に対する行列方程式を作成． */
  LinearEquation scale(const Eigen::VectorXd& scale) const;

  /* 変化率についての方程式を離散化して変化量についての方程式に変換． */
  LinearEquation discretise(const double& dt) const;

  friend std::ostream& operator<<(std::ostream& os, const LinearEquation& arg);
};

inline LinearEquation::LinearEquation(const Eigen::MatrixXd& _A, const Eigen::MatrixXd& _b) : A(_A), b(_b)
{
  assert(A.rows() == b.rows());
}

inline LinearEquation::LinearEquation(const Eigen::Index& var_size, const Eigen::Index& eq_size)
  : A(eq_size, var_size), b(eq_size)
{
}

inline LinearEquation::LinearEquation()
{
}

inline LinearEquation LinearEquation::Zero(const Eigen::Index& var_size, const Eigen::Index& eq_size)
{
  LinearEquation res(var_size, eq_size);
  res.setZero();
  return res;
}

inline void LinearEquation::resize(const Eigen::Index& var_size, const Eigen::Index& eq_size)
{
  A.conservativeResize(eq_size, var_size);
  b.conservativeResize(eq_size);
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

inline Eigen::Index LinearEquation::variableSize() const
{
  return A.cols();
}

inline Eigen::Index LinearEquation::equationSize() const
{
  return A.rows();
}
}  // namespace ctrl
