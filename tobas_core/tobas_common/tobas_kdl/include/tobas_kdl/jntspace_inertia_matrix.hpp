// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frames.hpp"
#include "./jacobian.hpp"
#include "./jntarray.hpp"

namespace tobas
{
namespace kdl
{
/* This class represents a fixed `nj` matrix containing the Joint-Space Inertia Matrix of a `kdl::Chain`. */
class JntSpaceInertiaMatrix
{
public:
  Eigen::MatrixXd data;

  inline explicit JntSpaceInertiaMatrix();
  inline explicit JntSpaceInertiaMatrix(int nj);

  /* Resize the array. */
  inline void resize(size_t nj);

  inline double operator()(size_t i, size_t j) const;
  inline double& operator()(size_t i, size_t j);

  inline size_t rows() const;
  inline size_t columns() const;

  inline friend void
  Add(const JntSpaceInertiaMatrix& src1, const JntSpaceInertiaMatrix& src2, JntSpaceInertiaMatrix& dest);
  inline friend void
  Subtract(const JntSpaceInertiaMatrix& src1, const JntSpaceInertiaMatrix& src2, JntSpaceInertiaMatrix& dest);
  inline friend void Multiply(const JntSpaceInertiaMatrix& src, const double& factor, JntSpaceInertiaMatrix& dest);
  inline friend void Divide(const JntSpaceInertiaMatrix& src, const double& factor, JntSpaceInertiaMatrix& dest);
  inline friend void Multiply(const JntSpaceInertiaMatrix& src, const JntArray& vec, JntArray& dest);
};

inline JntSpaceInertiaMatrix::JntSpaceInertiaMatrix()
{
}

inline JntSpaceInertiaMatrix::JntSpaceInertiaMatrix(int nj) : data(nj, nj)
{
  data.setZero();
}

inline void JntSpaceInertiaMatrix::resize(size_t nj)
{
  data.resize(nj, nj);
}

inline double JntSpaceInertiaMatrix::operator()(size_t i, size_t j) const
{
  return data(i, j);
}

inline double& JntSpaceInertiaMatrix::operator()(size_t i, size_t j)
{
  return data(i, j);
}

inline size_t JntSpaceInertiaMatrix::rows() const
{
  return static_cast<size_t>(data.rows());
}

inline size_t JntSpaceInertiaMatrix::columns() const
{
  return static_cast<size_t>(data.cols());
}

inline void Add(const JntSpaceInertiaMatrix& src1, const JntSpaceInertiaMatrix& src2, JntSpaceInertiaMatrix& dest)
{
  dest.data = src1.data + src2.data;
}

inline void Subtract(const JntSpaceInertiaMatrix& src1, const JntSpaceInertiaMatrix& src2, JntSpaceInertiaMatrix& dest)
{
  dest.data = src1.data - src2.data;
}

inline void Multiply(const JntSpaceInertiaMatrix& src, const double& factor, JntSpaceInertiaMatrix& dest)
{
  dest.data = factor * src.data;
}

inline void Divide(const JntSpaceInertiaMatrix& src, const double& factor, JntSpaceInertiaMatrix& dest)
{
  dest.data = src.data / factor;
}

inline void Multiply(const JntSpaceInertiaMatrix& src, const JntArray& vec, JntArray& dest)
{
  dest.data = src.data.lazyProduct(vec.data);
}
}  // namespace kdl
}  // namespace tobas
