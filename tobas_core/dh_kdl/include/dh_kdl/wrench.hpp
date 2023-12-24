#pragma once

#include "./vector.hpp"

namespace KDL
{
class Wrench;
using Wrenches = std::vector<Wrench>;
using WrenchMap = std::map<std::string, Wrench>;

class Wrench
{
public:
  Vector force;   // Force that is applied at the origin of the current ref frame
  Vector torque;  // Torque that is applied at the origin of the current ref frame

  // Does not initialise force and torque to zero via the underlying constructor of Vector
  inline explicit Wrench();
  inline explicit Wrench(const Vector& _force, const Vector& _torque);

  // @return a zero Wrench
  inline static Wrench Zero();

  // Changes the reference point of the wrench.
  // The vector p is expressed in the same base as the twist.
  // The vector p is a vector from the old point to the new point.
  inline Wrench refPoint(const Vector& p) const;

  // index-based access to components, first force(0..2), then torque(3..5)
  inline double operator()(size_t index) const;
  inline double& operator()(size_t index);

  inline Wrench& operator+=(const Wrench& arg);
  inline Wrench& operator-=(const Wrench& arg);

  inline friend Wrench operator-(const Wrench& arg);
  inline friend Wrench operator*(const Wrench& lhs, double rhs);
  inline friend Wrench operator*(double lhs, const Wrench& rhs);
  inline friend Wrench operator/(const Wrench& lhs, double rhs);
  inline friend Wrench operator+(const Wrench& lhs, const Wrench& rhs);
  inline friend Wrench operator-(const Wrench& lhs, const Wrench& rhs);

  inline friend void setToZero(Wrench& v);

  friend std::ostream& operator<<(std::ostream& os, const Wrench& arg);
};

inline Wrench::Wrench()
{
}

inline Wrench::Wrench(const Vector& _force, const Vector& _torque) : force(_force), torque(_torque)
{
}

inline Wrench Wrench::Zero()
{
  return Wrench(Vector::Zero(), Vector::Zero());
}

inline Wrench Wrench::refPoint(const Vector& p) const
{
  return Wrench(force, torque + force * p);
}

inline double Wrench::operator()(size_t index) const
{
  return index < 3 ? force(index) : torque(index - 3);
}

inline double& Wrench::operator()(size_t index)
{
  return index < 3 ? force(index) : torque(index - 3);
}

inline Wrench& Wrench::operator+=(const Wrench& arg)
{
  torque += arg.torque;
  force += arg.force;
  return *this;
}

inline Wrench& Wrench::operator-=(const Wrench& arg)
{
  torque -= arg.torque;
  force -= arg.force;
  return *this;
}

inline Wrench operator-(const Wrench& arg)
{
  return Wrench(-arg.force, -arg.torque);
}

inline Wrench operator*(const Wrench& lhs, double rhs)
{
  return Wrench(lhs.force * rhs, lhs.torque * rhs);
}

inline Wrench operator*(double lhs, const Wrench& rhs)
{
  return Wrench(lhs * rhs.force, lhs * rhs.torque);
}

inline Wrench operator/(const Wrench& lhs, double rhs)
{
  return Wrench(lhs.force / rhs, lhs.torque / rhs);
}

inline Wrench operator+(const Wrench& lhs, const Wrench& rhs)
{
  return Wrench(lhs.force + rhs.force, lhs.torque + rhs.torque);
}

inline Wrench operator-(const Wrench& lhs, const Wrench& rhs)
{
  return Wrench(lhs.force - rhs.force, lhs.torque - rhs.torque);
}

inline void setToZero(Wrench& v)
{
  setToZero(v.force);
  setToZero(v.torque);
}
}  // namespace KDL
