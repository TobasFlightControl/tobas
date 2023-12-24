#pragma once

#include <Eigen/Core>
#include <QuadProg++/Array.hh>

namespace quadprogpp
{
template <typename Derived>
void matrixEigenToQp(const Eigen::MatrixBase<Derived>& e, Matrix<double>& q)
{
  if (q.nrows() != e.rows() || q.ncols() != e.cols())
  {
    q.resize(e.rows(), e.cols());
  }

  for (size_t i = 0; i < e.rows(); ++i)
  {
    for (size_t j = 0; j < e.cols(); ++j)
    {
      q[i][j] = e(i, j);
    }
  }
}

template <typename Derived>
void vectorEigenToQp(const Eigen::MatrixBase<Derived>& e, Vector<double>& q)
{
  assert(e.cols() == 1);

  if (q.size() != e.size())
  {
    q.resize(e.rows());
  }

  for (size_t i = 0; i < e.rows(); ++i)
  {
    q[i] = e(i);
  }
}

template <typename Derived>
void matrixQpToEigen(const Matrix<double>& q, Eigen::MatrixBase<Derived>& e)
{
  // Eigenは安易にresizeできないため，引数の時点でサイズが合っていることを確認する
  assert(e.rows() == q.nrows() && e.cols() == q.ncols());

  for (size_t i = 0; i < e.rows(); ++i)
  {
    for (size_t j = 0; j < e.cols(); ++j)
    {
      e(i, j) = q[i][j];
    }
  }
}

template <typename Derived>
void vectorQpToEigen(const Vector<double>& q, Eigen::MatrixBase<Derived>& e)
{
  assert(e.rows() == q.size());
  assert(e.cols() == 1);

  for (size_t i = 0; i < e.rows(); ++i)
  {
    e(i) = q[i];
  }
}
}  // namespace quadprogpp
