#pragma once

#include "./frames.hpp"
#include "./utilities/utility.hpp"
#include "./utilities/rall1d.hpp"
#include "./utilities/traits.hpp"

template <>
struct Traits<KDL::doubleVel>
{
  typedef double valueType;
  typedef KDL::doubleVel derivType;
};
namespace KDL
{
class VectorVel
{
public:
  Vector p;  // position vector
  Vector v;  // velocity vector

  explicit VectorVel() : p(), v()
  {
  }

  explicit VectorVel(const Vector& _p, const Vector& _v) : p(_p), v(_v)
  {
  }

  explicit VectorVel(const Vector& _p) : p(_p), v(Vector::Zero())
  {
  }

  Vector value() const
  {
    return p;
  }

  Vector deriv() const
  {
    return v;
  }

  inline VectorVel& operator+=(const VectorVel& arg);
  inline VectorVel& operator-=(const VectorVel& arg);
  inline static VectorVel Zero();
  inline doubleVel norm(double eps = kDefaultEpsilon) const;
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
  inline friend void setToZero(VectorVel& v);

  inline friend VectorVel operator-(const VectorVel& r);
};

class RotationVel
{
public:
  Rotation R;  // Rotation matrix
  Vector w;    // rotation vector

  explicit RotationVel() : R(), w()
  {
  }

  explicit RotationVel(const Rotation& _R) : R(_R), w(Vector::Zero())
  {
  }

  explicit RotationVel(const Rotation& _R, const Vector& _w) : R(_R), w(_w)
  {
  }

  Rotation value() const
  {
    return R;
  }

  Vector deriv() const
  {
    return w;
  }

  inline VectorVel UnitX() const;
  inline VectorVel UnitY() const;
  inline VectorVel UnitZ() const;
  inline static RotationVel Identity();
  inline RotationVel inverse() const;
  inline VectorVel inverse(const VectorVel& arg) const;
  inline VectorVel inverse(const Vector& arg) const;
  inline VectorVel operator*(const VectorVel& arg) const;
  inline VectorVel operator*(const Vector& arg) const;
  inline void doRotX(const doubleVel& angle);
  inline void doRotY(const doubleVel& angle);
  inline void doRotZ(const doubleVel& angle);
  inline static RotationVel RotX(const doubleVel& angle);
  inline static RotationVel RotY(const doubleVel& angle);
  inline static RotationVel RotZ(const doubleVel& angle);
  inline static RotationVel Rot(const Vector& rotvec, const doubleVel& angle);
  // rotvec has arbitrary norm
  // rotation around a constant vector !
  inline static RotationVel Rot2(const Vector& rotvec, const doubleVel& angle);
  // rotvec is normalized.
  // rotation around a constant vector !
  inline friend RotationVel operator*(const RotationVel& r1, const RotationVel& r2);
  inline friend RotationVel operator*(const Rotation& r1, const RotationVel& r2);
  inline friend RotationVel operator*(const RotationVel& r1, const Rotation& r2);
};

class FrameVel
{
public:
  RotationVel M;
  VectorVel p;

  explicit FrameVel()
  {
  }

  explicit FrameVel(const Frame& _T) : M(_T.M), p(_T.p)
  {
  }

  explicit FrameVel(const Frame& _T, const Twist& _t) : M(_T.M, _t.rot), p(_T.p, _t.vel)
  {
  }

  explicit FrameVel(const RotationVel& _M, const VectorVel& _p) : M(_M), p(_p)
  {
  }

  Frame value() const
  {
    return Frame(M.value(), p.value());
  }

  Twist deriv() const
  {
    return Twist(p.deriv(), M.deriv());
  }

  inline static FrameVel Identity();
  inline FrameVel inverse() const;
  inline VectorVel inverse(const VectorVel& arg) const;
  inline VectorVel operator*(const VectorVel& arg) const;
  inline VectorVel operator*(const Vector& arg) const;
  inline VectorVel inverse(const Vector& arg) const;
  inline Frame getFrame() const;
  inline Twist getTwist() const;
  inline friend FrameVel operator*(const FrameVel& f1, const FrameVel& f2);
  inline friend FrameVel operator*(const Frame& f1, const FrameVel& f2);
  inline friend FrameVel operator*(const FrameVel& f1, const Frame& f2);
};

FrameVel FrameVel::Identity()
{
  return FrameVel(RotationVel::Identity(), VectorVel::Zero());
}

FrameVel operator*(const FrameVel& lhs, const FrameVel& rhs)
{
  return FrameVel(lhs.M * rhs.M, lhs.M * rhs.p + lhs.p);
}

FrameVel operator*(const FrameVel& lhs, const Frame& rhs)
{
  return FrameVel(lhs.M * rhs.M, lhs.M * rhs.p + lhs.p);
}

FrameVel operator*(const Frame& lhs, const FrameVel& rhs)
{
  return FrameVel(lhs.M * rhs.M, lhs.M * rhs.p + lhs.p);
}

VectorVel FrameVel::operator*(const VectorVel& arg) const
{
  return M * arg + p;
}

VectorVel FrameVel::operator*(const Vector& arg) const
{
  return M * arg + p;
}

VectorVel FrameVel::inverse(const VectorVel& arg) const
{
  return M.inverse(arg - p);
}

VectorVel FrameVel::inverse(const Vector& arg) const
{
  return M.inverse(arg - p);
}

FrameVel FrameVel::inverse() const
{
  return FrameVel(M.inverse(), -M.inverse(p));
}

Frame FrameVel::getFrame() const
{
  return Frame(M.R, p.p);
}

Twist FrameVel::getTwist() const
{
  return Twist(p.v, M.w);
}

RotationVel operator*(const RotationVel& r1, const RotationVel& r2)
{
  return RotationVel(r1.R * r2.R, r1.w + r1.R * r2.w);
}

RotationVel operator*(const Rotation& r1, const RotationVel& r2)
{
  return RotationVel(r1 * r2.R, r1 * r2.w);
}

RotationVel operator*(const RotationVel& r1, const Rotation& r2)
{
  return RotationVel(r1.R * r2, r1.w);
}

VectorVel RotationVel::UnitX() const
{
  return VectorVel(R.UnitX(), w * R.UnitX());
}

VectorVel RotationVel::UnitY() const
{
  return VectorVel(R.UnitY(), w * R.UnitY());
}

VectorVel RotationVel::UnitZ() const
{
  return VectorVel(R.UnitZ(), w * R.UnitZ());
}

RotationVel RotationVel::Identity()
{
  return RotationVel(Rotation::Identity(), Vector::Zero());
}

RotationVel RotationVel::inverse() const
{
  return RotationVel(R.inverse(), -R.inverse(w));
}

VectorVel RotationVel::inverse(const VectorVel& arg) const
{
  Vector tmp = R.inverse(arg.p);
  return VectorVel(tmp, R.inverse(arg.v - w * arg.p));
}

VectorVel RotationVel::inverse(const Vector& arg) const
{
  Vector tmp = R.inverse(arg);
  return VectorVel(tmp, R.inverse(-w * arg));
}

VectorVel RotationVel::operator*(const VectorVel& arg) const
{
  Vector tmp = R * arg.p;
  return VectorVel(tmp, w * tmp + R * arg.v);
}

VectorVel RotationVel::operator*(const Vector& arg) const
{
  Vector tmp = R * arg;
  return VectorVel(tmp, w * tmp);
}

void RotationVel::doRotX(const doubleVel& angle)
{
  w += R * Vector(angle.grad, 0, 0);
  R.doRotX(angle.t);
}

RotationVel RotationVel::RotX(const doubleVel& angle)
{
  return RotationVel(Rotation::RotX(angle.t), Vector(angle.grad, 0, 0));
}

void RotationVel::doRotY(const doubleVel& angle)
{
  w += R * Vector(0, angle.grad, 0);
  R.doRotY(angle.t);
}

RotationVel RotationVel::RotY(const doubleVel& angle)
{
  return RotationVel(Rotation::RotX(angle.t), Vector(0, angle.grad, 0));
}

void RotationVel::doRotZ(const doubleVel& angle)
{
  w += R * Vector(0, 0, angle.grad);
  R.doRotZ(angle.t);
}

RotationVel RotationVel::RotZ(const doubleVel& angle)
{
  return RotationVel(Rotation::RotZ(angle.t), Vector(0, 0, angle.grad));
}

RotationVel RotationVel::Rot(const Vector& rotvec, const doubleVel& angle)
{
  const auto v = rotvec.normalized();
  return RotationVel(Rotation::Rot2(v, angle.t), v * angle.grad);
}

RotationVel RotationVel::Rot2(const Vector& rotvec, const doubleVel& angle)
{
  return RotationVel(Rotation::Rot2(rotvec, angle.t), rotvec * angle.grad);
}

VectorVel operator+(const VectorVel& r1, const VectorVel& r2)
{
  return VectorVel(r1.p + r2.p, r1.v + r2.v);
}

VectorVel operator-(const VectorVel& r1, const VectorVel& r2)
{
  return VectorVel(r1.p - r2.p, r1.v - r2.v);
}

VectorVel operator+(const VectorVel& r1, const Vector& r2)
{
  return VectorVel(r1.p + r2, r1.v);
}

VectorVel operator-(const VectorVel& r1, const Vector& r2)
{
  return VectorVel(r1.p - r2, r1.v);
}

VectorVel operator+(const Vector& r1, const VectorVel& r2)
{
  return VectorVel(r1 + r2.p, r2.v);
}

VectorVel operator-(const Vector& r1, const VectorVel& r2)
{
  return VectorVel(r1 - r2.p, -r2.v);
}

VectorVel operator-(const VectorVel& r)
{
  return VectorVel(-r.p, -r.v);
}

void setToZero(VectorVel& v)
{
  setToZero(v.p);
  setToZero(v.v);
}

VectorVel operator*(const VectorVel& r1, const VectorVel& r2)
{
  return VectorVel(r1.p * r2.p, r1.p * r2.v + r1.v * r2.p);
}

VectorVel operator*(const VectorVel& r1, const Vector& r2)
{
  return VectorVel(r1.p * r2, r1.v * r2);
}

VectorVel operator*(const Vector& r1, const VectorVel& r2)
{
  return VectorVel(r1 * r2.p, r1 * r2.v);
}

VectorVel operator*(double r1, const VectorVel& r2)
{
  return VectorVel(r1 * r2.p, r1 * r2.v);
}

VectorVel operator*(const VectorVel& r1, double r2)
{
  return VectorVel(r1.p * r2, r1.v * r2);
}

VectorVel operator*(const doubleVel& r1, const VectorVel& r2)
{
  return VectorVel(r1.t * r2.p, r1.t * r2.v + r1.grad * r2.p);
}

VectorVel operator*(const VectorVel& r2, const doubleVel& r1)
{
  return VectorVel(r1.t * r2.p, r1.t * r2.v + r1.grad * r2.p);
}

VectorVel operator/(const VectorVel& r1, double r2)
{
  return VectorVel(r1.p / r2, r1.v / r2);
}

VectorVel operator/(const VectorVel& r2, const doubleVel& r1)
{
  return VectorVel(r2.p / r1.t, r2.v / r1.t - r2.p * r1.grad / r1.t / r1.t);
}

VectorVel operator*(const Rotation& R, const VectorVel& x)
{
  return VectorVel(R * x.p, R * x.v);
}

VectorVel& VectorVel::operator+=(const VectorVel& arg)
{
  p += arg.p;
  v += arg.v;
  return *this;
}

VectorVel& VectorVel::operator-=(const VectorVel& arg)
{
  p -= arg.p;
  v -= arg.v;
  return *this;
}

VectorVel VectorVel::Zero()
{
  return VectorVel(Vector::Zero(), Vector::Zero());
}

doubleVel VectorVel::norm(double eps) const
{
  double n = p.norm(eps);
  if (n < eps)  // Setting norm  of p and v to 0 in case norm of p is smaller than eps
    return doubleVel(0, 0);
  return doubleVel(n, p.dot(v) / n);
}
}  // namespace KDL
