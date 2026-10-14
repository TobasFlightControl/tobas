// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./twist_acc.hpp"

namespace tobas::kdl
{
class RotationAcc
{
public:
  Rotation R;  ///< rotation matrix
  Vector w;    ///< angular velocity vector
  Vector dw;   ///< Angular acceleration vector.

  inline explicit RotationAcc();
  inline explicit RotationAcc(const Rotation& _r);
  inline explicit RotationAcc(const Rotation& _r, const Vector& _w, const Vector& _dw);

  static inline RotationAcc Identity();

  inline void setIdentity();
  inline void setNaN();

  inline RotationAcc inverse() const;
  inline VectorAcc inverse(const Vector& _arg) const;
  inline VectorAcc inverse(const VectorAcc& _arg) const;
  inline TwistAcc inverse(const Twist& _arg) const;
  inline TwistAcc inverse(const TwistAcc& _arg) const;

  inline VectorAcc operator*(const Vector& _rhs) const;
  inline VectorAcc operator*(const VectorAcc& _rhs) const;
  inline TwistAcc operator*(const Twist& _rhs) const;
  inline TwistAcc operator*(const TwistAcc& _rhs) const;
  inline RotationAcc operator*(const Rotation& _rhs) const;
  inline RotationAcc operator*(const RotationAcc& _rhs) const;

  inline friend RotationAcc operator*(const Rotation& _lhs, const RotationAcc& _rhs);
};

inline RotationAcc::RotationAcc() : R(), w()
{
}

inline RotationAcc::RotationAcc(const Rotation& _r) : R(_r), w(Vector::Zero())
{
}

inline RotationAcc::RotationAcc(const Rotation& _r, const Vector& _w, const Vector& _dw) : R(_r), w(_w), dw(_dw)
{
}

inline RotationAcc RotationAcc::Identity()
{
  return RotationAcc(Rotation::Identity(), Vector::Zero(), Vector::Zero());
}

inline void RotationAcc::setIdentity()
{
  R.setIdentity();
  w.setZero();
  dw.setZero();
}

inline void RotationAcc::setNaN()
{
  R.setNaN();
  w.setNaN();
  dw.setNaN();
}

inline RotationAcc RotationAcc::inverse() const
{
  return RotationAcc(R.inverse(), -R.inverse(w), -R.inverse(dw));
}

inline VectorAcc RotationAcc::inverse(const Vector& _arg) const
{
  VectorAcc tmp;
  tmp.p = R.inverse(_arg);
  tmp.v = R.inverse(-w * _arg);
  tmp.dv = R.inverse(-dw * _arg - w * (R * tmp.v));
  return tmp;
}

inline VectorAcc RotationAcc::inverse(const VectorAcc& _arg) const
{
  VectorAcc tmp;
  tmp.p = R.inverse(_arg.p);
  tmp.v = R.inverse(_arg.v - w * _arg.p);
  tmp.dv = R.inverse(_arg.dv - dw * _arg.p - w * (_arg.v + R * tmp.v));
  return tmp;
}

inline TwistAcc RotationAcc::inverse(const Twist& _arg) const
{
  return TwistAcc(inverse(_arg.vel), inverse(_arg.rot));
}

inline TwistAcc RotationAcc::inverse(const TwistAcc& _arg) const
{
  return TwistAcc(inverse(_arg.vel), inverse(_arg.rot));
}

inline VectorAcc RotationAcc::operator*(const Vector& _rhs) const
{
  VectorAcc tmp;
  tmp.p = R * _rhs;
  tmp.v = w * tmp.p;
  tmp.dv = dw * tmp.p + w * tmp.v;
  return tmp;
}

inline VectorAcc RotationAcc::operator*(const VectorAcc& _rhs) const
{
  VectorAcc tmp;
  tmp.p = R * _rhs.p;
  tmp.dv = R * _rhs.v;
  tmp.v = w * tmp.p + tmp.dv;
  tmp.dv = dw * tmp.p + w * (tmp.v + tmp.dv) + R * _rhs.dv;
  return tmp;
}

inline TwistAcc RotationAcc::operator*(const Twist& _rhs) const
{
  return TwistAcc((*this) * _rhs.vel, (*this) * _rhs.rot);
}

inline TwistAcc RotationAcc::operator*(const TwistAcc& _rhs) const
{
  return TwistAcc((*this) * _rhs.vel, (*this) * _rhs.rot);
}

inline RotationAcc RotationAcc::operator*(const Rotation& _rhs) const
{
  return RotationAcc(R * _rhs, w, dw);
}

inline RotationAcc RotationAcc::operator*(const RotationAcc& _rhs) const
{
  return RotationAcc(R * _rhs.R, w + R * _rhs.w, dw + w * (R * _rhs.w) + R * _rhs.dw);
}

inline RotationAcc operator*(const Rotation& _lhs, const RotationAcc& _rhs)
{
  return RotationAcc(_lhs * _rhs.R, _lhs * _rhs.w, _lhs * _rhs.dw);
}
}  // namespace tobas::kdl
