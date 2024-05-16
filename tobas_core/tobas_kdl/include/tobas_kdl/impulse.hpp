#pragma once

#include "./twist.hpp"
#include "./wrench.hpp"

namespace tobas_kdl
{
class Impulse
{
public:
  Vector linear;   // [Ns]
  Vector angular;  // [Nms]

  inline explicit Impulse();
  inline explicit Impulse(const Vector& linear, const Vector& angular);

  /// Spatial cross product for 6d force vectors, beware all of them have to be expressed in the
  /// same reference frame/point
  inline friend Wrench operator*(const Twist& t, const Impulse& P);
};

inline Impulse::Impulse()
{
}

inline Impulse::Impulse(const Vector& _linear, const Vector& _angular)
  : linear(_linear), angular(_angular)
{
}

inline Wrench operator*(const Twist& t, const Impulse& P)
{
  return Wrench(t.rot * P.linear, t.rot * P.angular + t.vel * P.linear);
}
}  // namespace tobas_kdl
