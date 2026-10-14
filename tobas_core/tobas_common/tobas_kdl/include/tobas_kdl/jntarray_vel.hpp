// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frame_vel.hpp"
#include "./jntarray.hpp"
#include "./utilities/utility.hpp"

namespace tobas::kdl
{
class JntArrayVel
{
public:
  JntArray q;
  JntArray qd;

  inline explicit JntArrayVel();
  inline explicit JntArrayVel(size_t _nj);
  inline explicit JntArrayVel(const JntArray& _q, const JntArray& _qd);
  inline explicit JntArrayVel(const JntArray& _q);

  inline void resize(size_t _nj);
  inline void setZero();

  inline JntArrayVel operator+(const JntArrayVel& _rhs) const;
  inline JntArrayVel operator-(const JntArrayVel& _rhs) const;
  inline JntArrayVel operator*(double _rhs) const;
  inline JntArrayVel operator*(const doubleVel& _rhs) const;
  inline JntArrayVel operator/(double _rhs) const;
  inline JntArrayVel operator/(const doubleVel& _rhs) const;
};

inline JntArrayVel::JntArrayVel()
{
}

inline JntArrayVel::JntArrayVel(size_t _nj) : q(_nj), qd(_nj)
{
}

inline JntArrayVel::JntArrayVel(const JntArray& _q, const JntArray& _qd) : q(_q), qd(_qd)
{
  assert(q.size() == qd.size());
}

inline JntArrayVel::JntArrayVel(const JntArray& _q) : q(_q), qd(q.size())
{
}

inline void JntArrayVel::resize(size_t _nj)
{
  q.resize(_nj);
  qd.resize(_nj);
}

inline void JntArrayVel::setZero()
{
  q.setZero();
  qd.setZero();
}

inline JntArrayVel JntArrayVel::operator+(const JntArrayVel& _rhs) const
{
  return JntArrayVel(q + _rhs.q, qd + _rhs.qd);
}

inline JntArrayVel JntArrayVel::operator-(const JntArrayVel& _rhs) const
{
  return JntArrayVel(q - _rhs.q, qd - _rhs.qd);
}

inline JntArrayVel JntArrayVel::operator*(double _rhs) const
{
  return JntArrayVel(q * _rhs, qd * _rhs);
}

inline JntArrayVel JntArrayVel::operator*(const doubleVel& _rhs) const
{
  return JntArrayVel(q * _rhs.t, qd * _rhs.t + q * _rhs.grad);
}

inline JntArrayVel JntArrayVel::operator/(double _rhs) const
{
  assert(_rhs != 0);
  return JntArrayVel(q / _rhs, qd / _rhs);
}

inline JntArrayVel JntArrayVel::operator/(const doubleVel& _rhs) const
{
  assert(_rhs.t != 0);
  return JntArrayVel(q / _rhs.t, qd / _rhs.t - q * (_rhs.grad / sqr(_rhs.t)));
}
}  // namespace tobas::kdl
