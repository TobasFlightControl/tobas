// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frame.hpp"
#include "./rotation_vel.hpp"
#include "./vector_vel.hpp"

namespace tobas
{
namespace kdl
{
class FrameVel
{
public:
  VectorVel p;    ///< Position and linear velocity
  RotationVel M;  ///< Rotation and angular velocity

  inline explicit FrameVel();
  inline explicit FrameVel(const Frame& _t);
  inline explicit FrameVel(const Frame& _frame, const Twist& _twist);
  inline explicit FrameVel(const RotationVel& _m, const VectorVel& _p);

  static inline FrameVel Identity();

  inline void setIdentity();
  inline void setNaN();

  inline Frame getFrame() const;
  inline Twist getTwist() const;
  inline void setFrame(const Frame& _t);
  inline void setTwist(const Twist& _t);

  inline FrameVel inverse() const;
  inline VectorVel inverse(const Vector& _arg) const;
  inline VectorVel inverse(const VectorVel& _arg) const;

  inline VectorVel operator*(const Vector& _rhs) const;
  inline VectorVel operator*(const VectorVel& _rhs) const;

  inline FrameVel operator*(const Frame& _rhs) const;
  inline FrameVel operator*(const FrameVel& _rhs) const;

  inline friend FrameVel operator*(const Frame& _lhs, const FrameVel& _rhs);
};

inline FrameVel::FrameVel()
{
}

inline FrameVel::FrameVel(const Frame& _t) : p(_t.p), M(_t.M)
{
}

inline FrameVel::FrameVel(const Frame& _frame, const Twist& _twist) : p(_frame.p, _twist.vel), M(_frame.M, _twist.rot)
{
}

inline FrameVel::FrameVel(const RotationVel& _m, const VectorVel& _p) : p(_p), M(_m)
{
}

inline FrameVel FrameVel::Identity()
{
  return FrameVel(RotationVel::Identity(), VectorVel::Zero());
}

inline void FrameVel::setIdentity()
{
  p.setZero();
  M.setIdentity();
}

inline void FrameVel::setNaN()
{
  p.setNaN();
  M.setNaN();
}

inline Frame FrameVel::getFrame() const
{
  return Frame(M.R, p.p);
}

inline Twist FrameVel::getTwist() const
{
  return Twist(p.v, M.w);
}

inline void FrameVel::setFrame(const Frame& _t)
{
  p.p = _t.p;
  M.R = _t.M;
}

inline void FrameVel::setTwist(const Twist& _t)
{
  p.v = _t.vel;
  M.w = _t.rot;
}

inline FrameVel FrameVel::inverse() const
{
  return FrameVel(M.inverse(), -M.inverse(p));
}

inline VectorVel FrameVel::inverse(const Vector& _arg) const
{
  return M.inverse(_arg - p);
}

inline VectorVel FrameVel::inverse(const VectorVel& _arg) const
{
  return M.inverse(_arg - p);
}

inline VectorVel FrameVel::operator*(const Vector& _rhs) const
{
  return M * _rhs + p;
}

inline VectorVel FrameVel::operator*(const VectorVel& _rhs) const
{
  return M * _rhs + p;
}

inline FrameVel FrameVel::operator*(const Frame& _rhs) const
{
  return FrameVel(M * _rhs.M, M * _rhs.p + p);
}

inline FrameVel FrameVel::operator*(const FrameVel& _rhs) const
{
  return FrameVel(M * _rhs.M, M * _rhs.p + p);
}

inline FrameVel operator*(const Frame& _lhs, const FrameVel& _rhs)
{
  return FrameVel(_lhs.M * _rhs.M, _lhs.M * _rhs.p + _lhs.p);
}
}  // namespace kdl
}  // namespace tobas
