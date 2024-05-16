#pragma once

#include "./twistacc.hpp"

namespace KDL
{
class RotationAcc
{
public:
  Rotation R;  // rotation matrix
  Vector w;    // angular velocity vector
  Vector dw;   // angular acceration vector

  inline explicit RotationAcc();
  inline explicit RotationAcc(const Rotation& _R);
  inline explicit RotationAcc(const Rotation& _R, const Vector& _w, const Vector& _dw);

  inline static RotationAcc Identity();

  inline RotationAcc inverse() const;
  inline VectorAcc inverse(const Vector& arg) const;
  inline VectorAcc inverse(const VectorAcc& arg) const;
  inline TwistAcc inverse(const Twist& arg) const;
  inline TwistAcc inverse(const TwistAcc& arg) const;

  inline VectorAcc operator*(const Vector& rhs) const;
  inline VectorAcc operator*(const VectorAcc& rhs) const;
  inline TwistAcc operator*(const Twist& rhs) const;
  inline TwistAcc operator*(const TwistAcc& rhs) const;
  inline RotationAcc operator*(const Rotation& rhs) const;
  inline RotationAcc operator*(const RotationAcc& rhs) const;

  inline friend RotationAcc operator*(const Rotation& lhs, const RotationAcc& rhs);
};

inline RotationAcc::RotationAcc() : R(), w()
{
}

inline RotationAcc::RotationAcc(const Rotation& _R) : R(_R), w(Vector::Zero())
{
}

inline RotationAcc::RotationAcc(const Rotation& _R, const Vector& _w, const Vector& _dw)
  : R(_R), w(_w), dw(_dw)
{
}

inline RotationAcc RotationAcc::Identity()
{
  return RotationAcc(Rotation::Identity(), Vector::Zero(), Vector::Zero());
}

inline RotationAcc RotationAcc::inverse() const
{
  return RotationAcc(R.inverse(), -R.inverse(w), -R.inverse(dw));
}

inline VectorAcc RotationAcc::inverse(const Vector& arg) const
{
  VectorAcc tmp;
  tmp.p = R.inverse(arg);
  tmp.v = R.inverse(-w * arg);
  tmp.dv = R.inverse(-dw * arg - w * (R * tmp.v));
  return tmp;
}

inline VectorAcc RotationAcc::inverse(const VectorAcc& arg) const
{
  VectorAcc tmp;
  tmp.p = R.inverse(arg.p);
  tmp.v = R.inverse(arg.v - w * arg.p);
  tmp.dv = R.inverse(arg.dv - dw * arg.p - w * (arg.v + R * tmp.v));
  return tmp;
}

inline TwistAcc RotationAcc::inverse(const Twist& arg) const
{
  return TwistAcc(inverse(arg.vel), inverse(arg.rot));
}

inline TwistAcc RotationAcc::inverse(const TwistAcc& arg) const
{
  return TwistAcc(inverse(arg.vel), inverse(arg.rot));
}

inline VectorAcc RotationAcc::operator*(const Vector& rhs) const
{
  VectorAcc tmp;
  tmp.p = R * rhs;
  tmp.v = w * tmp.p;
  tmp.dv = dw * tmp.p + w * tmp.v;
  return tmp;
}

inline VectorAcc RotationAcc::operator*(const VectorAcc& rhs) const
{
  VectorAcc tmp;
  tmp.p = R * rhs.p;
  tmp.dv = R * rhs.v;
  tmp.v = w * tmp.p + tmp.dv;
  tmp.dv = dw * tmp.p + w * (tmp.v + tmp.dv) + R * rhs.dv;
  return tmp;
}

inline TwistAcc RotationAcc::operator*(const Twist& rhs) const
{
  return TwistAcc((*this) * rhs.vel, (*this) * rhs.rot);
}

inline TwistAcc RotationAcc::operator*(const TwistAcc& rhs) const
{
  return TwistAcc((*this) * rhs.vel, (*this) * rhs.rot);
}

inline RotationAcc RotationAcc::operator*(const Rotation& rhs) const
{
  return RotationAcc(R * rhs, w, dw);
}

inline RotationAcc RotationAcc::operator*(const RotationAcc& rhs) const
{
  return RotationAcc(R * rhs.R, w + R * rhs.w, dw + w * (R * rhs.w) + R * rhs.dw);
}

inline RotationAcc operator*(const Rotation& lhs, const RotationAcc& rhs)
{
  return RotationAcc(lhs * rhs.R, lhs * rhs.w, lhs * rhs.dw);
}
}  // namespace KDL
