#pragma once

#include "./vector.hpp"
#include "./rotation.hpp"
#include "./utilities/rall1d.hpp"

namespace tobas_kdl
{
class VectorVel
{
public:
  Vector p;  // position vector
  Vector v;  // velocity vector

  inline explicit VectorVel();
  inline explicit VectorVel(const Vector& _p, const Vector& _v);
  inline explicit VectorVel(const Vector& _p);

  inline static VectorVel Zero();

  doubleVel norm(double eps = kDefaultEpsilon) const;

  VectorVel& operator+=(const VectorVel& arg);
  VectorVel& operator-=(const VectorVel& arg);

  friend void setToZero(VectorVel& v);

  inline friend VectorVel operator+(const VectorVel& r1, const VectorVel& r2);
  inline friend VectorVel operator-(const VectorVel& r1, const VectorVel& r2);
  inline friend VectorVel operator+(const Vector& r1, const VectorVel& r2);
  inline friend VectorVel operator-(const Vector& r1, const VectorVel& r2);
  inline friend VectorVel operator+(const VectorVel& r1, const Vector& r2);
  inline friend VectorVel operator-(const VectorVel& r1, const Vector& r2);
  inline friend VectorVel operator*(const VectorVel& r1, const VectorVel& r2);
  inline friend VectorVel operator*(const VectorVel& r1, const Vector& r2);
  inline friend VectorVel operator*(const Vector& r1, const VectorVel& r2);
  inline friend VectorVel operator*(const VectorVel& r1, double r2);
  inline friend VectorVel operator*(double r1, const VectorVel& r2);
  inline friend VectorVel operator*(const doubleVel& r1, const VectorVel& r2);
  inline friend VectorVel operator*(const VectorVel& r2, const doubleVel& r1);
  inline friend VectorVel operator*(const Rotation& R, const VectorVel& x);

  inline friend VectorVel operator/(const VectorVel& r1, double r2);
  inline friend VectorVel operator/(const VectorVel& r2, const doubleVel& r1);

  inline friend VectorVel operator-(const VectorVel& r);
};

inline VectorVel::VectorVel()
{
}

inline VectorVel::VectorVel(const Vector& _p, const Vector& _v) : p(_p), v(_v)
{
}

inline VectorVel::VectorVel(const Vector& _p) : p(_p), v(Vector::Zero())
{
}

VectorVel VectorVel::Zero()
{
  return VectorVel(Vector::Zero(), Vector::Zero());
}

inline VectorVel operator+(const VectorVel& r1, const VectorVel& r2)
{
  return VectorVel(r1.p + r2.p, r1.v + r2.v);
}

inline VectorVel operator-(const VectorVel& r1, const VectorVel& r2)
{
  return VectorVel(r1.p - r2.p, r1.v - r2.v);
}

inline VectorVel operator+(const VectorVel& r1, const Vector& r2)
{
  return VectorVel(r1.p + r2, r1.v);
}

inline VectorVel operator-(const VectorVel& r1, const Vector& r2)
{
  return VectorVel(r1.p - r2, r1.v);
}

inline VectorVel operator+(const Vector& r1, const VectorVel& r2)
{
  return VectorVel(r1 + r2.p, r2.v);
}

inline VectorVel operator-(const Vector& r1, const VectorVel& r2)
{
  return VectorVel(r1 - r2.p, -r2.v);
}

inline VectorVel operator-(const VectorVel& r)
{
  return VectorVel(-r.p, -r.v);
}

inline VectorVel operator*(const VectorVel& r1, const VectorVel& r2)
{
  return VectorVel(r1.p * r2.p, r1.p * r2.v + r1.v * r2.p);
}

inline VectorVel operator*(const VectorVel& r1, const Vector& r2)
{
  return VectorVel(r1.p * r2, r1.v * r2);
}

inline VectorVel operator*(const Vector& r1, const VectorVel& r2)
{
  return VectorVel(r1 * r2.p, r1 * r2.v);
}

inline VectorVel operator*(double r1, const VectorVel& r2)
{
  return VectorVel(r1 * r2.p, r1 * r2.v);
}

inline VectorVel operator*(const VectorVel& r1, double r2)
{
  return VectorVel(r1.p * r2, r1.v * r2);
}

inline VectorVel operator*(const doubleVel& r1, const VectorVel& r2)
{
  return VectorVel(r1.t * r2.p, r1.t * r2.v + r1.grad * r2.p);
}

inline VectorVel operator*(const VectorVel& r2, const doubleVel& r1)
{
  return VectorVel(r1.t * r2.p, r1.t * r2.v + r1.grad * r2.p);
}

inline VectorVel operator/(const VectorVel& r1, double r2)
{
  return VectorVel(r1.p / r2, r1.v / r2);
}

inline VectorVel operator/(const VectorVel& r2, const doubleVel& r1)
{
  return VectorVel(r2.p / r1.t, r2.v / r1.t - r2.p * r1.grad / r1.t / r1.t);
}

inline VectorVel operator*(const Rotation& R, const VectorVel& x)
{
  return VectorVel(R * x.p, R * x.v);
}
}  // namespace tobas_kdl
