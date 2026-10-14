// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frame.hpp"
#include "./rotation_acc.hpp"
#include "./twist_acc.hpp"
#include "./vector_acc.hpp"

namespace tobas::kdl
{
class FrameAcc
{
public:
  VectorAcc p;    ///< Translation, velocity and acceleration of origin.
  RotationAcc M;  ///< Rotation,angular velocity, and angular acceleration of frame.

  inline explicit FrameAcc();
  inline explicit FrameAcc(const Frame& _t);
  inline explicit FrameAcc(const Frame& _frame, const Twist& _twist, const Twist& _dt);
  inline explicit FrameAcc(const RotationAcc& _m, const VectorAcc& _p);

  static inline FrameAcc Identity();

  inline void setIdentity();
  inline void setNaN();

  inline Frame getFrame() const;
  inline Twist getTwist() const;
  inline Accel getAccel() const;

  inline FrameAcc inverse() const;
  inline VectorAcc inverse(const Vector& _arg) const;
  inline VectorAcc inverse(const VectorAcc& _arg) const;
  inline TwistAcc inverse(const Twist& _arg) const;
  inline TwistAcc inverse(const TwistAcc& _arg) const;

  inline VectorAcc operator*(const Vector& _rhs) const;
  inline VectorAcc operator*(const VectorAcc& _rhs) const;
  inline TwistAcc operator*(const Twist& _rhs) const;
  inline TwistAcc operator*(const TwistAcc& _rhs) const;
  inline FrameAcc operator*(const Frame& _rhs) const;
  inline FrameAcc operator*(const FrameAcc& _rhs) const;

  inline friend FrameAcc operator*(const Frame& _lhs, const FrameAcc& _rhs);
};

inline FrameAcc::FrameAcc()
{
}

inline FrameAcc::FrameAcc(const Frame& _t) : p(_t.p), M(_t.M)
{
}

inline FrameAcc::FrameAcc(const Frame& _frame, const Twist& _twist, const Twist& _dt)
  : p(_frame.p, _twist.vel, _dt.vel), M(_frame.M, _twist.rot, _dt.rot)
{
}

inline FrameAcc::FrameAcc(const RotationAcc& _m, const VectorAcc& _p) : p(_p), M(_m)
{
}

inline FrameAcc FrameAcc::Identity()
{
  return FrameAcc(RotationAcc::Identity(), VectorAcc::Zero());
}

inline void FrameAcc::setIdentity()
{
  p.setZero();
  M.setIdentity();
}

inline void FrameAcc::setNaN()
{
  p.setNaN();
  M.setNaN();
}

inline Frame FrameAcc::getFrame() const
{
  return Frame(M.R, p.p);
}

inline Twist FrameAcc::getTwist() const
{
  return Twist(p.v, M.w);
}

inline Accel FrameAcc::getAccel() const
{
  return Accel(p.dv, M.dw);
}

inline FrameAcc FrameAcc::inverse() const
{
  return FrameAcc(M.inverse(), -M.inverse(p));
}

inline VectorAcc FrameAcc::inverse(const Vector& _arg) const
{
  return M.inverse(_arg - p);
}

inline VectorAcc FrameAcc::inverse(const VectorAcc& _arg) const
{
  return M.inverse(_arg - p);
}

inline TwistAcc FrameAcc::inverse(const Twist& _arg) const
{
  TwistAcc tmp;
  tmp.rot = M.inverse(_arg.rot);
  tmp.vel = M.inverse(_arg.vel - p * _arg.rot);
  return tmp;
}

inline TwistAcc FrameAcc::inverse(const TwistAcc& _arg) const
{
  TwistAcc tmp;
  tmp.rot = M.inverse(_arg.rot);
  tmp.vel = M.inverse(_arg.vel - p * _arg.rot);
  return tmp;
}

inline VectorAcc FrameAcc::operator*(const Vector& _rhs) const
{
  return M * _rhs + p;
}

inline VectorAcc FrameAcc::operator*(const VectorAcc& _rhs) const
{
  return M * _rhs + p;
}

inline TwistAcc FrameAcc::operator*(const Twist& _rhs) const
{
  TwistAcc tmp;
  tmp.rot = M * _rhs.rot;
  tmp.vel = M * _rhs.vel + p * tmp.rot;
  return tmp;
}

inline TwistAcc FrameAcc::operator*(const TwistAcc& _rhs) const
{
  TwistAcc tmp;
  tmp.rot = M * _rhs.rot;
  tmp.vel = M * _rhs.vel + p * tmp.rot;
  return tmp;
}

inline FrameAcc FrameAcc::operator*(const Frame& _rhs) const
{
  return FrameAcc(M * _rhs.M, M * _rhs.p + p);
}

inline FrameAcc FrameAcc::operator*(const FrameAcc& _rhs) const
{
  return FrameAcc(M * _rhs.M, M * _rhs.p + p);
}

inline FrameAcc operator*(const Frame& _lhs, const FrameAcc& _rhs)
{
  return FrameAcc(_lhs.M * _rhs.M, _lhs.M * _rhs.p + _lhs.p);
}
}  // namespace tobas::kdl
