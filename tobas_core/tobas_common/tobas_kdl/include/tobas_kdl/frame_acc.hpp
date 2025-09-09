#pragma once

#include "./frame.hpp"
#include "./rotation_acc.hpp"
#include "./twist_acc.hpp"
#include "./vector_acc.hpp"

namespace kdl
{
class FrameAcc
{
public:
  VectorAcc p;    // Translation, velocity and acceleration of origin.
  RotationAcc M;  // Rotation,angular velocity, and angular acceleration of frame.

  inline explicit FrameAcc();
  inline explicit FrameAcc(const Frame& _T);
  inline explicit FrameAcc(const Frame& _T, const Twist& _t, const Twist& _dt);
  inline explicit FrameAcc(const RotationAcc& _M, const VectorAcc& _p);

  static inline FrameAcc Identity();

  inline void setIdentity();

  inline Frame getFrame() const;
  inline Twist getTwist() const;
  inline Accel getAccel() const;

  inline FrameAcc inverse() const;
  inline VectorAcc inverse(const Vector& arg) const;
  inline VectorAcc inverse(const VectorAcc& arg) const;
  inline TwistAcc inverse(const Twist& arg) const;
  inline TwistAcc inverse(const TwistAcc& arg) const;

  inline VectorAcc operator*(const Vector& rhs) const;
  inline VectorAcc operator*(const VectorAcc& rhs) const;
  inline TwistAcc operator*(const Twist& rhs) const;
  inline TwistAcc operator*(const TwistAcc& rhs) const;
  inline FrameAcc operator*(const Frame& rhs) const;
  inline FrameAcc operator*(const FrameAcc& rhs) const;

  inline friend FrameAcc operator*(const Frame& lhs, const FrameAcc& rhs);
};

inline FrameAcc::FrameAcc()
{
}

inline FrameAcc::FrameAcc(const Frame& _T) : p(_T.p), M(_T.M)
{
}

inline FrameAcc::FrameAcc(const Frame& _T, const Twist& _t, const Twist& _dt)
  : p(_T.p, _t.vel, _dt.vel), M(_T.M, _t.rot, _dt.rot)
{
}

inline FrameAcc::FrameAcc(const RotationAcc& _M, const VectorAcc& _p) : p(_p), M(_M)
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

inline VectorAcc FrameAcc::inverse(const Vector& arg) const
{
  return M.inverse(arg - p);
}

inline VectorAcc FrameAcc::inverse(const VectorAcc& arg) const
{
  return M.inverse(arg - p);
}

inline TwistAcc FrameAcc::inverse(const Twist& arg) const
{
  TwistAcc tmp;
  tmp.rot = M.inverse(arg.rot);
  tmp.vel = M.inverse(arg.vel - p * arg.rot);
  return tmp;
}

inline TwistAcc FrameAcc::inverse(const TwistAcc& arg) const
{
  TwistAcc tmp;
  tmp.rot = M.inverse(arg.rot);
  tmp.vel = M.inverse(arg.vel - p * arg.rot);
  return tmp;
}

inline VectorAcc FrameAcc::operator*(const Vector& rhs) const
{
  return M * rhs + p;
}

inline VectorAcc FrameAcc::operator*(const VectorAcc& rhs) const
{
  return M * rhs + p;
}

inline TwistAcc FrameAcc::operator*(const Twist& rhs) const
{
  TwistAcc tmp;
  tmp.rot = M * rhs.rot;
  tmp.vel = M * rhs.vel + p * tmp.rot;
  return tmp;
}

inline TwistAcc FrameAcc::operator*(const TwistAcc& rhs) const
{
  TwistAcc tmp;
  tmp.rot = M * rhs.rot;
  tmp.vel = M * rhs.vel + p * tmp.rot;
  return tmp;
}

inline FrameAcc FrameAcc::operator*(const Frame& rhs) const
{
  return FrameAcc(M * rhs.M, M * rhs.p + p);
}

inline FrameAcc FrameAcc::operator*(const FrameAcc& rhs) const
{
  return FrameAcc(M * rhs.M, M * rhs.p + p);
}

inline FrameAcc operator*(const Frame& lhs, const FrameAcc& rhs)
{
  return FrameAcc(lhs.M * rhs.M, lhs.M * rhs.p + lhs.p);
}
}  // namespace kdl
