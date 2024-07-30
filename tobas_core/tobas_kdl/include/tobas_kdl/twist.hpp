#pragma once

#include <tobas_eigen_tools/typedef.hpp>

#include "./accel.hpp"
#include "./wrench.hpp"

namespace kdl
{
class Twist;
using TwistMap = std::map<std::string, Twist>;

/**
 * \brief represents both translational and rotational velocities.
 *
 * This class represents a twist. A twist is the combination of translational
 * velocity and rotational velocity applied at one point.
 */
class Twist
{
public:
  Vector vel;  // The linear velocity of that point
  Vector rot;  // The angular velocity of that point

  inline explicit Twist();
  inline explicit Twist(const Vector& vel, const Vector& rot);

  // @return a zero Twist : Twist(Vector::Zero(), Vector::Zero())
  inline static Twist Zero();

  // Changes the reference point of the twist.
  // The vector p is expressed in the same base as the twist.
  // The vector p is a vector from the old point to the new point.
  // 自フレームAと同じ剛体中のフレームBのツイストを求める．
  inline Twist refPoint(const Vector& p) const;

  // index-based access to components, first vel(0..2), then rot(3..5)
  inline double operator()(size_t i) const;
  // index-based access to components, first vel(0..2), then rot(3..5)
  inline double& operator()(size_t i);

  inline Twist& operator+=(const Twist& arg);
  inline Twist& operator-=(const Twist& arg);

  inline friend Twist operator-(const Twist& arg);
  inline friend Twist operator*(const Twist& lhs, double rhs);
  inline friend Twist operator*(double lhs, const Twist& rhs);
  inline friend Twist operator/(const Twist& lhs, double rhs);
  inline friend Twist operator+(const Twist& lhs, const Twist& rhs);
  inline friend Twist operator-(const Twist& lhs, const Twist& rhs);

  // Spatial cross product for 6d motion vectors,
  // beware all of them have to be expressed in the same reference frame
  inline friend Accel operator*(const Twist& lhs, const Twist& rhs);

  Eigen::Vector6d ravel() const;

  friend std::ostream& operator<<(std::ostream& os, const Twist& arg);
};

inline Twist::Twist()
{
}

inline Twist::Twist(const Vector& _vel, const Vector& _rot) : vel(_vel), rot(_rot)
{
}

inline Twist Twist::Zero()
{
  return Twist(Vector::Zero(), Vector::Zero());
}

inline Twist Twist::refPoint(const Vector& p) const
{
  return Twist(vel + rot * p, rot);
}

inline double Twist::operator()(size_t i) const
{
  return i < 3 ? vel(i) : rot(i - 3);
}

inline double& Twist::operator()(size_t i)
{
  return i < 3 ? vel(i) : rot(i - 3);
}

inline Twist& Twist::operator+=(const Twist& arg)
{
  vel += arg.vel;
  rot += arg.rot;
  return *this;
}

inline Twist& Twist::operator-=(const Twist& arg)
{
  vel -= arg.vel;
  rot -= arg.rot;
  return *this;
}

inline Twist operator-(const Twist& arg)
{
  return Twist(-arg.vel, -arg.rot);
}

inline Twist operator*(const Twist& lhs, double rhs)
{
  return Twist(lhs.vel * rhs, lhs.rot * rhs);
}

inline Twist operator*(double lhs, const Twist& rhs)
{
  return Twist(lhs * rhs.vel, lhs * rhs.rot);
}

inline Twist operator/(const Twist& lhs, double rhs)
{
  return Twist(lhs.vel / rhs, lhs.rot / rhs);
}

inline Twist operator+(const Twist& lhs, const Twist& rhs)
{
  return Twist(lhs.vel + rhs.vel, lhs.rot + rhs.rot);
}

inline Twist operator-(const Twist& lhs, const Twist& rhs)
{
  return Twist(lhs.vel - rhs.vel, lhs.rot - rhs.rot);
}

inline Accel operator*(const Twist& lhs, const Twist& rhs)
{
  return Accel(lhs.rot * rhs.vel + lhs.vel * rhs.rot, lhs.rot * rhs.rot);
}
}  // namespace kdl
