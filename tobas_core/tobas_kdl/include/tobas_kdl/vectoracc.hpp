#pragma once

#include "./vector.hpp"
#include "./rotation.hpp"
#include "./utilities/rall2d.hpp"

namespace kdl
{
class VectorAcc
{
public:
  Vector p;   // position vector
  Vector v;   // velocity vector
  Vector dv;  // acceleration vector

  inline explicit VectorAcc();
  inline explicit VectorAcc(const Vector& _p);
  inline explicit VectorAcc(const Vector& _p, const Vector& _v);
  inline explicit VectorAcc(const Vector& _p, const Vector& _v, const Vector& _dv);

  inline static VectorAcc Zero();

  inline void setZero();

  inline doubleAcc norm() const;

  inline doubleAcc dot(const Vector& rhs) const;
  inline doubleAcc dot(const VectorAcc& rhs) const;

  inline VectorAcc& operator+=(const VectorAcc& arg);
  inline VectorAcc& operator-=(const VectorAcc& arg);

  inline friend VectorAcc operator+(const VectorAcc& r1, const VectorAcc& r2);
  inline friend VectorAcc operator-(const VectorAcc& r1, const VectorAcc& r2);
  inline friend VectorAcc operator+(const Vector& r1, const VectorAcc& r2);
  inline friend VectorAcc operator-(const Vector& r1, const VectorAcc& r2);
  inline friend VectorAcc operator+(const VectorAcc& r1, const Vector& r2);
  inline friend VectorAcc operator-(const VectorAcc& r1, const Vector& r2);
  /* cross prod. */
  inline friend VectorAcc operator*(const VectorAcc& r1, const VectorAcc& r2);
  inline friend VectorAcc operator*(const VectorAcc& r1, const Vector& r2);
  inline friend VectorAcc operator*(const Vector& r1, const VectorAcc& r2);
  inline friend VectorAcc operator*(const VectorAcc& r1, double r2);
  /* scalar mult. */
  inline friend VectorAcc operator*(double r1, const VectorAcc& r2);
  inline friend VectorAcc operator*(const doubleAcc& r1, const VectorAcc& r2);
  inline friend VectorAcc operator*(const VectorAcc& r2, const doubleAcc& r1);
  inline friend VectorAcc operator*(const Rotation& R, const VectorAcc& x);
  inline friend VectorAcc operator/(const VectorAcc& r1, double r2);
  inline friend VectorAcc operator/(const VectorAcc& r2, const doubleAcc& r1);
  /* unary - */
  inline friend VectorAcc operator-(const VectorAcc& r);
};

inline VectorAcc::VectorAcc()
{
}

inline VectorAcc::VectorAcc(const Vector& _p) : p(_p), v(Vector::Zero()), dv(Vector::Zero())
{
}

inline VectorAcc::VectorAcc(const Vector& _p, const Vector& _v) : p(_p), v(_v), dv(Vector::Zero())
{
}

inline VectorAcc::VectorAcc(const Vector& _p, const Vector& _v, const Vector& _dv) : p(_p), v(_v), dv(_dv)
{
}

inline VectorAcc VectorAcc::Zero()
{
  return VectorAcc(Vector::Zero(), Vector::Zero(), Vector::Zero());
}

inline void VectorAcc::setZero()
{
  p.setZero();
  v.setZero();
  dv.setZero();
}

inline doubleAcc VectorAcc::norm() const
{
  doubleAcc res;
  res.t = p.norm();
  res.d = p.dot(v) / res.t;
  res.dd = (p.dot(dv) + v.dot(v) - res.d * res.d) / res.t;
  return res;
}

inline doubleAcc VectorAcc::dot(const Vector& rhs) const
{
  return doubleAcc(p.dot(rhs), v.dot(rhs), dv.dot(rhs));
}

inline doubleAcc VectorAcc::dot(const VectorAcc& rhs) const
{
  return doubleAcc(p.dot(rhs.p), p.dot(rhs.v) + v.dot(rhs.p), p.dot(rhs.dv) + 2 * v.dot(rhs.v) + dv.dot(rhs.p));
}

inline VectorAcc& VectorAcc::operator+=(const VectorAcc& arg)
{
  p += arg.p;
  v += arg.v;
  dv += arg.dv;
  return *this;
}
inline VectorAcc& VectorAcc::operator-=(const VectorAcc& arg)
{
  p -= arg.p;
  v -= arg.v;
  dv -= arg.dv;
  return *this;
}

inline VectorAcc operator+(const VectorAcc& r1, const VectorAcc& r2)
{
  return VectorAcc(r1.p + r2.p, r1.v + r2.v, r1.dv + r2.dv);
}

inline VectorAcc operator-(const VectorAcc& r1, const VectorAcc& r2)
{
  return VectorAcc(r1.p - r2.p, r1.v - r2.v, r1.dv - r2.dv);
}
inline VectorAcc operator+(const Vector& r1, const VectorAcc& r2)
{
  return VectorAcc(r1 + r2.p, r2.v, r2.dv);
}

inline VectorAcc operator-(const Vector& r1, const VectorAcc& r2)
{
  return VectorAcc(r1 - r2.p, -r2.v, -r2.dv);
}

inline VectorAcc operator+(const VectorAcc& r1, const Vector& r2)
{
  return VectorAcc(r1.p + r2, r1.v, r1.dv);
}

inline VectorAcc operator-(const VectorAcc& r1, const Vector& r2)
{
  return VectorAcc(r1.p - r2, r1.v, r1.dv);
}

inline VectorAcc operator-(const VectorAcc& r)
{
  return VectorAcc(-r.p, -r.v, -r.dv);
}

inline VectorAcc operator*(const VectorAcc& r1, const VectorAcc& r2)
{
  return VectorAcc(r1.p * r2.p, r1.p * r2.v + r1.v * r2.p, r1.dv * r2.p + 2 * r1.v * r2.v + r1.p * r2.dv);
}

inline VectorAcc operator*(const VectorAcc& r1, const Vector& r2)
{
  return VectorAcc(r1.p * r2, r1.v * r2, r1.dv * r2);
}

inline VectorAcc operator*(const Vector& r1, const VectorAcc& r2)
{
  return VectorAcc(r1 * r2.p, r1 * r2.v, r1 * r2.dv);
}

inline VectorAcc operator*(double r1, const VectorAcc& r2)
{
  return VectorAcc(r1 * r2.p, r1 * r2.v, r1 * r2.dv);
}

inline VectorAcc operator*(const VectorAcc& r1, double r2)
{
  return VectorAcc(r1.p * r2, r1.v * r2, r1.dv * r2);
}

inline VectorAcc operator*(const doubleAcc& r1, const VectorAcc& r2)
{
  return VectorAcc(r1.t * r2.p, r1.t * r2.v + r1.d * r2.p, r1.t * r2.dv + 2 * r1.d * r2.v + r1.dd * r2.p);
}

inline VectorAcc operator*(const VectorAcc& r2, const doubleAcc& r1)
{
  return VectorAcc(r1.t * r2.p, r1.t * r2.v + r1.d * r2.p, r1.t * r2.dv + 2 * r1.d * r2.v + r1.dd * r2.p);
}

inline VectorAcc operator*(const Rotation& R, const VectorAcc& x)
{
  return VectorAcc(R * x.p, R * x.v, R * x.dv);
}

inline VectorAcc operator/(const VectorAcc& r1, double r2)
{
  assert(r2 != 0.);
  return VectorAcc(r1.p / r2, r1.v / r2, r1.dv / r2);
}

inline VectorAcc operator/(const VectorAcc& r2, const doubleAcc& r1)
{
  return r2 * (1. / r1);
}
}  // namespace kdl
