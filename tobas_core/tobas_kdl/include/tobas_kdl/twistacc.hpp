#pragma once

#include "./twist.hpp"
#include "./accel.hpp"
#include "./vectoracc.hpp"

namespace KDL
{
class TwistAcc
{
public:
  VectorAcc vel;  // translational velocity and its 1st and 2nd derivative
  VectorAcc rot;  // rotational velocity and its 1st and 2nd derivative

  inline explicit TwistAcc();
  inline explicit TwistAcc(const VectorAcc& _vel, const VectorAcc& _rot);

  static inline TwistAcc Zero();

  // Changes the reference point of the TwistAcc.
  // The RVector v_base_AB is expressed in the same base as the TwistAcc
  // The RVector v_base_AB is a RVector from the old point to
  // the new point.
  // Complexity : 6M+6A
  inline TwistAcc refPoint(const VectorAcc& v_base_AB);

  inline Twist getTwist() const;
  inline Accel getAccel() const;

  inline TwistAcc& operator+=(const TwistAcc& arg);
  inline TwistAcc& operator-=(const TwistAcc& arg);

  inline friend TwistAcc operator*(const TwistAcc& lhs, double rhs);
  inline friend TwistAcc operator*(double lhs, const TwistAcc& rhs);
  inline friend TwistAcc operator/(const TwistAcc& lhs, double rhs);

  inline friend TwistAcc operator*(const TwistAcc& lhs, const doubleAcc& rhs);
  inline friend TwistAcc operator*(const doubleAcc& lhs, const TwistAcc& rhs);
  inline friend TwistAcc operator/(const TwistAcc& lhs, const doubleAcc& rhs);

  inline friend TwistAcc operator+(const TwistAcc& lhs, const TwistAcc& rhs);
  inline friend TwistAcc operator-(const TwistAcc& lhs, const TwistAcc& rhs);
  inline friend TwistAcc operator-(const TwistAcc& arg);

  friend class RotationAcc;
  friend class FrameAcc;
};

inline TwistAcc::TwistAcc()
{
}

inline TwistAcc::TwistAcc(const VectorAcc& _vel, const VectorAcc& _rot) : vel(_vel), rot(_rot)
{
}

inline TwistAcc TwistAcc::Zero()
{
  return TwistAcc(VectorAcc::Zero(), VectorAcc::Zero());
}

inline TwistAcc TwistAcc::refPoint(const VectorAcc& v_base_AB)
{
  return TwistAcc(this->vel + this->rot * v_base_AB, this->rot);
}

inline Twist TwistAcc::getTwist() const
{
  return Twist(vel.p, rot.p);
}

inline Accel TwistAcc::getAccel() const
{
  return Accel(vel.v, rot.v);
}

inline TwistAcc& TwistAcc::operator+=(const TwistAcc& arg)
{
  vel += arg.vel;
  rot += arg.rot;
  return *this;
}

inline TwistAcc& TwistAcc::operator-=(const TwistAcc& arg)
{
  vel -= arg.vel;
  rot -= arg.rot;
  return *this;
}

inline TwistAcc operator*(const TwistAcc& lhs, double rhs)
{
  return TwistAcc(lhs.vel * rhs, lhs.rot * rhs);
}

inline TwistAcc operator*(double lhs, const TwistAcc& rhs)
{
  return TwistAcc(lhs * rhs.vel, lhs * rhs.rot);
}

inline TwistAcc operator/(const TwistAcc& lhs, double rhs)
{
  return TwistAcc(lhs.vel / rhs, lhs.rot / rhs);
}

inline TwistAcc operator*(const TwistAcc& lhs, const doubleAcc& rhs)
{
  return TwistAcc(lhs.vel * rhs, lhs.rot * rhs);
}

inline TwistAcc operator*(const doubleAcc& lhs, const TwistAcc& rhs)
{
  return TwistAcc(lhs * rhs.vel, lhs * rhs.rot);
}

inline TwistAcc operator/(const TwistAcc& lhs, const doubleAcc& rhs)
{
  return TwistAcc(lhs.vel / rhs, lhs.rot / rhs);
}

inline TwistAcc operator+(const TwistAcc& lhs, const TwistAcc& rhs)
{
  return TwistAcc(lhs.vel + rhs.vel, lhs.rot + rhs.rot);
}

inline TwistAcc operator-(const TwistAcc& lhs, const TwistAcc& rhs)
{
  return TwistAcc(lhs.vel - rhs.vel, lhs.rot - rhs.rot);
}

inline TwistAcc operator-(const TwistAcc& arg)
{
  return TwistAcc(-arg.vel, -arg.rot);
}
}  // namespace KDL
