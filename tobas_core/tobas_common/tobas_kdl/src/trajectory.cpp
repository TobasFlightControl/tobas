// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl/trajectory.hpp"

#include <cassert>

#include <tobas_math/core.hpp>
#include <tobas_math/definitions.hpp>

namespace tobas::kdl
{
CycloidGenerator3d::CycloidGenerator3d()
{
}

void CycloidGenerator3d::generate(const kdl::Vector& _p0, const kdl::Vector& _pf, double _t, double _h, double _k)
{
  assert(_t > 0.0);
  assert(_h > 0.0);
  assert(_k >= 0.0);

  p0_ = _p0;
  pf_ = _pf;
  T_ = _t;
  h_ = _h;
  k_ = _k;
  TT_ = math::sqr(_t);
  kk_ = math::sqr(_k);
  p_diff_ = _pf - _p0;
}

void CycloidGenerator3d::get(double _t, const Rotation& _r, Vector& _p, Vector& _v, Vector& _a) const
{
  assert(_t >= 0.0);

  if (_t <= T_) {
    getPos(_t, _r, _p);
    getVel(_t, _r, _v);
    getAcc(_t, _r, _a);
  }
  else {
    _p = _r * pf_;
    _v.setZero();
    _a.setZero();
  }
}

void CycloidGenerator3d::get(double _t, Vector& _p, Vector& _v, Vector& _a) const
{
  get(_t, r0_, _p, _v, _a);
}

void CycloidGenerator3d::get(double _t, Vector& _p, Vector& _v) const
{
  Vector dummy_vector;
  get(_t, r0_, _p, _v, dummy_vector);
}

void CycloidGenerator3d::get(double _t, Vector& _p) const
{
  Vector dummy_vector;
  get(_t, r0_, _p, dummy_vector, dummy_vector);
}

void CycloidGenerator3d::getPos(double _t, const kdl::Rotation& _r, Vector& _p) const
{
  const auto theta = computeTheta(_t);
  const auto tmp = (theta - std::sin(theta)) / M_2PI;

  _p.x(p0_.x() + p_diff_.x() * tmp);
  _p.y(p0_.y() + p_diff_.y() * tmp);
  _p.z(pf_.z() + h_ / 2 * (1 - std::cos(theta)) - p_diff_.z() * std::exp(-k_ * _t / T_));

  _p = _r * _p;
}

void CycloidGenerator3d::getVel(double _t, const kdl::Rotation& _r, Vector& _v) const
{
  const auto theta = computeTheta(_t);
  const auto tmp = (1 - std::cos(theta)) / T_;

  _v.x(p_diff_.x() * tmp);
  _v.y(p_diff_.y() * tmp);
  _v.z(M_PI * h_ * std::sin(theta) / T_ + p_diff_.z() * k_ * std::exp(-k_ * _t / T_) / T_);

  _v = _r * _v;
}

void CycloidGenerator3d::getAcc(double _t, const kdl::Rotation& _r, Vector& _a) const
{
  const auto theta = computeTheta(_t);
  const auto tmp = M_2PI / TT_ * std::sin(theta);

  _a.x(p_diff_.x() * tmp);
  _a.y(p_diff_.y() * tmp);
  _a.z(2 * math::sqr(M_PI) * h_ / TT_ * std::cos(theta) - p_diff_.z() * kk_ / TT_ * std::exp(-k_ * _t / T_));

  _a = _r * _a;
}

double CycloidGenerator3d::computeTheta(double _t) const
{
  assert(_t >= 0.0);
  return M_2PI * _t / T_;
}
}  // namespace tobas::kdl
