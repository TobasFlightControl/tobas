// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cassert>
#include <cstddef>
#include <ostream>

#include <eigen3/Eigen/Core>

namespace tobas::kdl
{
class JntArray
{
public:
  Eigen::VectorXd data;

  inline explicit JntArray();
  inline explicit JntArray(size_t _nj);
  inline explicit JntArray(const Eigen::VectorXd& _data);

  static inline JntArray Zero(size_t _nj);
  static inline JntArray Constant(size_t _nj, double _value);

  inline void resize(size_t _nj);
  inline void setZero();

  inline size_t size() const;

  inline double max() const;
  inline double min() const;

  inline JntArray max(double _x);
  inline JntArray min(double _x);

  inline JntArray hadamard(const JntArray& _arg);

  inline double operator()(size_t _i) const;
  inline double& operator()(size_t _i);

  inline JntArray operator+(const JntArray& _rhs) const;
  inline JntArray operator-(const JntArray& _rhs) const;
  inline JntArray operator*(double _rhs) const;
  inline JntArray operator/(double _rhs) const;
  inline JntArray& operator+=(const JntArray& _rhs);
  inline JntArray& operator-=(const JntArray& _rhs);
  inline JntArray& operator*=(double _rhs);
  inline JntArray& operator/=(double _rhs);

  inline friend JntArray operator+(const JntArray& _arg);
  inline friend JntArray operator-(const JntArray& _arg);
  inline friend JntArray operator*(double _lhs, const JntArray& _rhs);

  inline friend std::ostream& operator<<(std::ostream& _os, const JntArray& _arg);
};

inline JntArray::JntArray()
{
}

inline JntArray::JntArray(size_t _nj) : data(_nj)
{
}

inline JntArray::JntArray(const Eigen::VectorXd& _data) : data(_data)
{
}

inline JntArray JntArray::Zero(size_t _nj)
{
  return JntArray(Eigen::VectorXd::Zero(_nj));
}

inline JntArray JntArray::Constant(size_t _nj, double _value)
{
  return JntArray(Eigen::VectorXd::Constant(_nj, _value));
}

inline void JntArray::resize(size_t _nj)
{
  data.conservativeResize(_nj);
}

void JntArray::setZero()
{
  data.setZero();
}

inline size_t JntArray::size() const
{
  return static_cast<size_t>(data.size());
}

inline double JntArray::max() const
{
  return data.maxCoeff();
}

inline double JntArray::min() const
{
  return data.minCoeff();
}

inline JntArray JntArray::max(double _x)
{
  return JntArray(data.cwiseMax(_x));
}

inline JntArray JntArray::min(double _x)
{
  return JntArray(data.cwiseMin(_x));
}

inline JntArray JntArray::hadamard(const JntArray& _arg)
{
  assert(size() == _arg.size());
  return JntArray(data.cwiseProduct(_arg.data));
}

inline double JntArray::operator()(size_t _i) const
{
  return data(_i);
}

inline double& JntArray::operator()(size_t _i)
{
  return data(_i);
}

inline JntArray JntArray::operator+(const JntArray& _rhs) const
{
  assert(size() == _rhs.size());
  return JntArray(data + _rhs.data);
}

inline JntArray JntArray::operator-(const JntArray& _rhs) const
{
  assert(size() == _rhs.size());
  return JntArray(data - _rhs.data);
}

inline JntArray JntArray::operator*(double _rhs) const
{
  return JntArray(data * _rhs);
}

inline JntArray JntArray::operator/(double _rhs) const
{
  assert(_rhs != 0);
  return JntArray(data / _rhs);
}

inline JntArray& JntArray::operator+=(const JntArray& _rhs)
{
  assert(size() == _rhs.size());
  data += _rhs.data;
  return *this;
}

inline JntArray& JntArray::operator-=(const JntArray& _rhs)
{
  assert(size() == _rhs.size());
  data -= _rhs.data;
  return *this;
}

inline JntArray& JntArray::operator*=(double _rhs)
{
  data *= _rhs;
  return *this;
}

inline JntArray& JntArray::operator/=(double _rhs)
{
  assert(_rhs != 0);
  data /= _rhs;
  return *this;
}

inline JntArray operator+(const JntArray& _arg)
{
  return _arg;
}

inline JntArray operator-(const JntArray& _arg)
{
  return JntArray(-_arg.data);
}

inline JntArray operator*(double _lhs, const JntArray& _rhs)
{
  return JntArray(_lhs * _rhs.data);
}

inline std::ostream& operator<<(std::ostream& _os, const JntArray& _arg)
{
  _os << _arg.data.transpose();
  return _os;
}
}  // namespace tobas::kdl
