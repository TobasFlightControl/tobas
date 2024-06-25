#pragma once

#include "./frame.hpp"
#include "./vectorvel.hpp"
#include "./rotationvel.hpp"

namespace kdl
{
class FrameVel
{
public:
  RotationVel M;  // Rotation and angular velocity
  VectorVel p;    // Position and linear velocity

  inline explicit FrameVel();
  inline explicit FrameVel(const Frame& _T);
  inline explicit FrameVel(const Frame& _T, const Twist& _t);
  inline explicit FrameVel(const RotationVel& _M, const VectorVel& _p);

  inline static FrameVel Identity();

  inline Frame getFrame() const;
  inline Twist getTwist() const;

  inline FrameVel inverse() const;
  inline VectorVel inverse(const Vector& arg) const;
  inline VectorVel inverse(const VectorVel& arg) const;

  inline VectorVel operator*(const Vector& rhs) const;
  inline VectorVel operator*(const VectorVel& rhs) const;

  inline FrameVel operator*(const Frame& rhs) const;
  inline FrameVel operator*(const FrameVel& rhs) const;

  inline friend FrameVel operator*(const Frame& lhs, const FrameVel& rhs);
};

inline FrameVel::FrameVel()
{
}

inline FrameVel::FrameVel(const Frame& _T) : M(_T.M), p(_T.p)
{
}

inline FrameVel::FrameVel(const Frame& _T, const Twist& _t) : M(_T.M, _t.rot), p(_T.p, _t.vel)
{
}

inline FrameVel::FrameVel(const RotationVel& _M, const VectorVel& _p) : M(_M), p(_p)
{
}

inline FrameVel FrameVel::Identity()
{
  return FrameVel(RotationVel::Identity(), VectorVel::Zero());
}

inline Frame FrameVel::getFrame() const
{
  return Frame(M.R, p.p);
}

inline Twist FrameVel::getTwist() const
{
  return Twist(p.v, M.w);
}

inline FrameVel FrameVel::inverse() const
{
  return FrameVel(M.inverse(), -M.inverse(p));
}

inline VectorVel FrameVel::inverse(const Vector& arg) const
{
  return M.inverse(arg - p);
}

inline VectorVel FrameVel::inverse(const VectorVel& arg) const
{
  return M.inverse(arg - p);
}

inline VectorVel FrameVel::operator*(const Vector& rhs) const
{
  return M * rhs + p;
}

inline VectorVel FrameVel::operator*(const VectorVel& rhs) const
{
  return M * rhs + p;
}

inline FrameVel FrameVel::operator*(const Frame& rhs) const
{
  return FrameVel(M * rhs.M, M * rhs.p + p);
}

inline FrameVel FrameVel::operator*(const FrameVel& rhs) const
{
  return FrameVel(M * rhs.M, M * rhs.p + p);
}

inline FrameVel operator*(const Frame& lhs, const FrameVel& rhs)
{
  return FrameVel(lhs.M * rhs.M, lhs.M * rhs.p + lhs.p);
}
}  // namespace kdl
