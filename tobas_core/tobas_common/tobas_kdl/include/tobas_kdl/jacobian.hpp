// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_eigen_tools/typedef.hpp>

#include "./frames.hpp"
#include "./jntarray.hpp"
#include "./segment_jacobian.hpp"

namespace tobas::kdl
{
class Jacobian;
using JacobianMap = std::map<std::string, Jacobian>;

class Jacobian
{
public:
  Eigen::Matrix6Xd data;

  inline explicit Jacobian();
  inline explicit Jacobian(size_t _nj);
  inline explicit Jacobian(const Eigen::Matrix6Xd& _data);

  static inline Jacobian Zero(size_t _nj);

  inline void resize(size_t _nj);
  inline void setZero();

  inline size_t rows() const;
  inline size_t columns() const;

  inline SegmentJacobian getColumn(size_t _i) const;
  inline void setColumn(size_t _i, const SegmentJacobian& _jac);

  inline void changeRefPoint(const Vector& _base_ab);
  inline void changeBase(const Rotation& _rot);
  inline void changeRefFrame(const Frame& _frame);

  inline double operator()(size_t _i, size_t _j) const;
  inline double& operator()(size_t _i, size_t _j);

  inline Twist operator*(const JntArray& _rhs) const;
};

inline Jacobian::Jacobian()
{
}

inline Jacobian::Jacobian(size_t _nj) : data(6, _nj)
{
}

inline Jacobian::Jacobian(const Eigen::Matrix6Xd& _data) : data(_data)
{
}

inline Jacobian Jacobian::Zero(size_t _nj)
{
  return Jacobian(Eigen::Matrix6Xd::Zero(Eigen::NoChange, _nj));
}

inline void Jacobian::resize(size_t _nj)
{
  data.conservativeResize(Eigen::NoChange, _nj);
}

inline void Jacobian::setZero()
{
  data.setZero();
}

inline size_t Jacobian::rows() const
{
  return static_cast<size_t>(data.rows());
}

inline size_t Jacobian::columns() const
{
  return static_cast<size_t>(data.cols());
}

inline SegmentJacobian Jacobian::getColumn(size_t _i) const
{
  return SegmentJacobian(Vector(data(0, _i), data(1, _i), data(2, _i)), Vector(data(3, _i), data(4, _i), data(5, _i)));
}

inline void Jacobian::setColumn(size_t _i, const SegmentJacobian& _jac)
{
  data.col(_i).head<3>() = _jac.linear.data;
  data.col(_i).tail<3>() = _jac.angular.data;
}

inline void Jacobian::changeRefPoint(const Vector& _base_ab)
{
  for (size_t i = 0; i < columns(); ++i) {
    setColumn(i, getColumn(i).refPoint(_base_ab));
  }
}

inline void Jacobian::changeBase(const Rotation& _rot)
{
  for (size_t i = 0; i < columns(); ++i) {
    setColumn(i, _rot * getColumn(i));
  }
}

inline void Jacobian::changeRefFrame(const Frame& _frame)
{
  for (size_t i = 0; i < columns(); ++i) {
    setColumn(i, _frame * getColumn(i));
  }
}

inline double Jacobian::operator()(size_t _i, size_t _j) const
{
  return data(_i, _j);
}

inline double& Jacobian::operator()(size_t _i, size_t _j)
{
  return data(_i, _j);
}

inline Twist Jacobian::operator*(const JntArray& _rhs) const
{
  const Eigen::Vector6d t = data * _rhs.data;
  return Twist(Vector(t.head<3>()), Vector(t.tail<3>()));
}
}  // namespace tobas::kdl
