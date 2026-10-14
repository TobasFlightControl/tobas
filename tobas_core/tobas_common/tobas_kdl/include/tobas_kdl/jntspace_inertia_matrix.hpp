// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>

#include <eigen3/Eigen/Core>

#include "./jntarray.hpp"

namespace tobas::kdl
{
/** This class represents a fixed `nj` matrix containing the joint-space inertia matrix. */
class JntSpaceInertiaMatrix
{
public:
  Eigen::MatrixXd data;

  inline explicit JntSpaceInertiaMatrix();
  inline explicit JntSpaceInertiaMatrix(int _nj);

  /** Resize the array. */
  inline void resize(size_t _nj);

  inline double operator()(size_t _i, size_t _j) const;
  inline double& operator()(size_t _i, size_t _j);

  inline size_t rows() const;
  inline size_t columns() const;

  inline friend void
  Add(const JntSpaceInertiaMatrix& _src1, const JntSpaceInertiaMatrix& _src2, JntSpaceInertiaMatrix& _dest);
  inline friend void
  Subtract(const JntSpaceInertiaMatrix& _src1, const JntSpaceInertiaMatrix& _src2, JntSpaceInertiaMatrix& _dest);
  inline friend void Multiply(const JntSpaceInertiaMatrix& _src, double _factor, JntSpaceInertiaMatrix& _dest);
  inline friend void Divide(const JntSpaceInertiaMatrix& _src, double _factor, JntSpaceInertiaMatrix& _dest);
  inline friend void Multiply(const JntSpaceInertiaMatrix& _src, const JntArray& _vec, JntArray& _dest);
};

inline JntSpaceInertiaMatrix::JntSpaceInertiaMatrix()
{
}

inline JntSpaceInertiaMatrix::JntSpaceInertiaMatrix(int _nj) : data(_nj, _nj)
{
  data.setZero();
}

inline void JntSpaceInertiaMatrix::resize(size_t _nj)
{
  data.resize(_nj, _nj);
}

inline double JntSpaceInertiaMatrix::operator()(size_t _i, size_t _j) const
{
  return data(_i, _j);
}

inline double& JntSpaceInertiaMatrix::operator()(size_t _i, size_t _j)
{
  return data(_i, _j);
}

inline size_t JntSpaceInertiaMatrix::rows() const
{
  return static_cast<size_t>(data.rows());
}

inline size_t JntSpaceInertiaMatrix::columns() const
{
  return static_cast<size_t>(data.cols());
}

inline void Add(const JntSpaceInertiaMatrix& _src1, const JntSpaceInertiaMatrix& _src2, JntSpaceInertiaMatrix& _dest)
{
  _dest.data = _src1.data + _src2.data;
}

inline void
Subtract(const JntSpaceInertiaMatrix& _src1, const JntSpaceInertiaMatrix& _src2, JntSpaceInertiaMatrix& _dest)
{
  _dest.data = _src1.data - _src2.data;
}

inline void Multiply(const JntSpaceInertiaMatrix& _src, double _factor, JntSpaceInertiaMatrix& _dest)
{
  _dest.data = _factor * _src.data;
}

inline void Divide(const JntSpaceInertiaMatrix& _src, double _factor, JntSpaceInertiaMatrix& _dest)
{
  _dest.data = _src.data / _factor;
}

inline void Multiply(const JntSpaceInertiaMatrix& _src, const JntArray& _vec, JntArray& _dest)
{
  _dest.data = _src.data.lazyProduct(_vec.data);
}
}  // namespace tobas::kdl
