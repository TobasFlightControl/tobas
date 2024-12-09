#pragma once

#include "./vector.hpp"
#include "./rotation.hpp"
#include "./utilities/rall1d.hpp"

namespace kdl
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

  inline void setZero();

  inline doubleVel norm() const;

  inline VectorVel& operator+=(const VectorVel& rhs);
  inline VectorVel& operator-=(const VectorVel& rhs);

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

inline void VectorVel::setZero()
{
  p.setZero();
  v.setZero();
}

inline doubleVel VectorVel::norm() const
{
  const auto n = p.norm();
  if (n < std::numeric_limits<double>::epsilon())
    return doubleVel(0, 0);
  return doubleVel(n, p.dot(v) / n);
}

inline VectorVel& VectorVel::operator+=(const VectorVel& rhs)
{
  p += rhs.p;
  v += rhs.v;
  return *this;
}

inline VectorVel& VectorVel::operator-=(const VectorVel& rhs)
{
  p -= rhs.p;
  v -= rhs.v;
  return *this;
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
  assert(r2 != 0);
  return VectorVel(r1.p / r2, r1.v / r2);
}

inline VectorVel operator/(const VectorVel& r2, const doubleVel& r1)
{
  assert(r1.t != 0);
  assert(r1.grad != 0);
  return VectorVel(r2.p / r1.t, r2.v / r1.t - r2.p * r1.grad / r1.t / r1.t);
}

inline VectorVel operator*(const Rotation& R, const VectorVel& x)
{
  return VectorVel(R * x.p, R * x.v);
}
}  // namespace kdl
