// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frame_vel.hpp"
#include "./jntarray.hpp"
#include "./utilities/utility.hpp"

namespace tobas
{
namespace kdl
{
class JntArrayVel
{
public:
  JntArray q;
  JntArray qd;

  inline explicit JntArrayVel();
  inline explicit JntArrayVel(size_t nj);
  inline explicit JntArrayVel(const JntArray& q, const JntArray& qd);
  inline explicit JntArrayVel(const JntArray& q);

  inline void resize(size_t nj);
  inline void setZero();

  inline JntArrayVel operator+(const JntArrayVel& rhs) const;
  inline JntArrayVel operator-(const JntArrayVel& rhs) const;
  inline JntArrayVel operator*(const double& rhs) const;
  inline JntArrayVel operator*(const doubleVel& rhs) const;
  inline JntArrayVel operator/(const double& rhs) const;
  inline JntArrayVel operator/(const doubleVel& rhs) const;
};

inline JntArrayVel::JntArrayVel()
{
}

inline JntArrayVel::JntArrayVel(size_t nj) : q(nj), qd(nj)
{
}

inline JntArrayVel::JntArrayVel(const JntArray& _q, const JntArray& _qd) : q(_q), qd(_qd)
{
  assert(q.size() == qd.size());
}

inline JntArrayVel::JntArrayVel(const JntArray& _q) : q(_q), qd(q.size())
{
}

inline void JntArrayVel::resize(size_t nj)
{
  q.resize(nj);
  qd.resize(nj);
}

inline void JntArrayVel::setZero()
{
  q.setZero();
  qd.setZero();
}

inline JntArrayVel JntArrayVel::operator+(const JntArrayVel& rhs) const
{
  return JntArrayVel(q + rhs.q, qd + rhs.qd);
}

inline JntArrayVel JntArrayVel::operator-(const JntArrayVel& rhs) const
{
  return JntArrayVel(q - rhs.q, qd - rhs.qd);
}

inline JntArrayVel JntArrayVel::operator*(const double& rhs) const
{
  return JntArrayVel(q * rhs, qd * rhs);
}

inline JntArrayVel JntArrayVel::operator*(const doubleVel& rhs) const
{
  return JntArrayVel(q * rhs.t, qd * rhs.t + q * rhs.grad);
}

inline JntArrayVel JntArrayVel::operator/(const double& rhs) const
{
  assert(rhs != 0);
  return JntArrayVel(q / rhs, qd / rhs);
}

inline JntArrayVel JntArrayVel::operator/(const doubleVel& rhs) const
{
  assert(rhs.t != 0);
  return JntArrayVel(q / rhs.t, qd / rhs.t - q * (rhs.grad / sqr(rhs.t)));
}
}  // namespace kdl
}  // namespace tobas
