// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./twist.hpp"
#include "./wrench.hpp"

namespace tobas::kdl
{
class Impulse
{
public:
  Vector linear;   ///< [Ns]
  Vector angular;  ///< [Nms]

  inline explicit Impulse();
  inline explicit Impulse(const Vector& _linear, const Vector& _angular);

  inline friend Wrench operator*(const Twist& _t, const Impulse& _p);
};

inline Impulse::Impulse()
{
}

inline Impulse::Impulse(const Vector& _linear, const Vector& _angular) : linear(_linear), angular(_angular)
{
}

inline Wrench operator*(const Twist& _t, const Impulse& _p)
{
  return Wrench(_t.rot * _p.linear, _t.rot * _p.angular + _t.vel * _p.linear);
}
}  // namespace tobas::kdl
