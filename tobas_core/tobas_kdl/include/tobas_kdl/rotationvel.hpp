#pragma once

#include "./vectorvel.hpp"

namespace kdl
{
class RotationVel
{
public:
  Rotation R;  // Rotation matrix
  Vector w;    // rotation vector

  inline explicit RotationVel();
  inline explicit RotationVel(const Rotation& _R);
  inline explicit RotationVel(const Rotation& _R, const Vector& _w);

  inline static RotationVel Identity();
  inline static RotationVel RotX(const doubleVel& angle);
  inline static RotationVel RotY(const doubleVel& angle);
  inline static RotationVel RotZ(const doubleVel& angle);
  /* rotvec has arbitrary norm. rotation around a constant vector! */
  inline static RotationVel Rot(const Vector& rotvec, const doubleVel& angle);
  /* rotvec is normalized. rotation around a constant vector! */
  inline static RotationVel Rot2(const Vector& rotvec, const doubleVel& angle);

  inline VectorVel UnitX() const;
  inline VectorVel UnitY() const;
  inline VectorVel UnitZ() const;
  inline RotationVel inverse() const;
  inline VectorVel inverse(const VectorVel& arg) const;
  inline VectorVel inverse(const Vector& arg) const;
  inline VectorVel operator*(const VectorVel& arg) const;
  inline VectorVel operator*(const Vector& arg) const;
  inline void doRotX(const doubleVel& angle);
  inline void doRotY(const doubleVel& angle);
  inline void doRotZ(const doubleVel& angle);

  inline friend RotationVel operator*(const RotationVel& r1, const RotationVel& r2);
  inline friend RotationVel operator*(const Rotation& r1, const RotationVel& r2);
  inline friend RotationVel operator*(const RotationVel& r1, const Rotation& r2);
};

inline RotationVel::RotationVel()
{
}

inline RotationVel::RotationVel(const Rotation& _R) : R(_R), w(Vector::Zero())
{
}

inline RotationVel::RotationVel(const Rotation& _R, const Vector& _w) : R(_R), w(_w)
{
}

inline RotationVel RotationVel::Identity()
{
  return RotationVel(Rotation::Identity(), Vector::Zero());
}

inline RotationVel RotationVel::RotX(const doubleVel& angle)
{
  return RotationVel(Rotation::RotX(angle.t), Vector(angle.grad, 0, 0));
}

inline void RotationVel::doRotY(const doubleVel& angle)
{
  w += R * Vector(0, angle.grad, 0);
  R.doRotY(angle.t);
}

inline RotationVel RotationVel::RotY(const doubleVel& angle)
{
  return RotationVel(Rotation::RotX(angle.t), Vector(0, angle.grad, 0));
}

inline void RotationVel::doRotZ(const doubleVel& angle)
{
  w += R * Vector(0, 0, angle.grad);
  R.doRotZ(angle.t);
}

inline RotationVel RotationVel::RotZ(const doubleVel& angle)
{
  return RotationVel(Rotation::RotZ(angle.t), Vector(0, 0, angle.grad));
}

inline RotationVel RotationVel::Rot(const Vector& rotvec, const doubleVel& angle)
{
  const auto v = rotvec.normalized();
  return RotationVel(Rotation::Rot2(v, angle.t), v * angle.grad);
}

inline RotationVel RotationVel::Rot2(const Vector& rotvec, const doubleVel& angle)
{
  return RotationVel(Rotation::Rot2(rotvec, angle.t), rotvec * angle.grad);
}

inline VectorVel RotationVel::UnitX() const
{
  return VectorVel(R.UnitX(), w * R.UnitX());
}

inline VectorVel RotationVel::UnitY() const
{
  return VectorVel(R.UnitY(), w * R.UnitY());
}

inline VectorVel RotationVel::UnitZ() const
{
  return VectorVel(R.UnitZ(), w * R.UnitZ());
}

inline RotationVel RotationVel::inverse() const
{
  return RotationVel(R.inverse(), -R.inverse(w));
}

inline VectorVel RotationVel::inverse(const VectorVel& arg) const
{
  Vector tmp = R.inverse(arg.p);
  return VectorVel(tmp, R.inverse(arg.v - w * arg.p));
}

inline VectorVel RotationVel::inverse(const Vector& arg) const
{
  Vector tmp = R.inverse(arg);
  return VectorVel(tmp, R.inverse(-w * arg));
}

inline VectorVel RotationVel::operator*(const VectorVel& arg) const
{
  Vector tmp = R * arg.p;
  return VectorVel(tmp, w * tmp + R * arg.v);
}

inline VectorVel RotationVel::operator*(const Vector& arg) const
{
  Vector tmp = R * arg;
  return VectorVel(tmp, w * tmp);
}

inline void RotationVel::doRotX(const doubleVel& angle)
{
  w += R * Vector(angle.grad, 0, 0);
  R.doRotX(angle.t);
}

inline RotationVel operator*(const RotationVel& r1, const RotationVel& r2)
{
  return RotationVel(r1.R * r2.R, r1.w + r1.R * r2.w);
}

inline RotationVel operator*(const Rotation& r1, const RotationVel& r2)
{
  return RotationVel(r1 * r2.R, r1 * r2.w);
}

inline RotationVel operator*(const RotationVel& r1, const Rotation& r2)
{
  return RotationVel(r1.R * r2, r1.w);
}
}  // namespace kdl
