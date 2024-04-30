#pragma once

#include "./utility.hpp"

namespace KDL
{
class Vector;
class Rotation;
class Frame;
class Twist;
class Wrench;
class doubleVel;
class VectorVel;
class RotationVel;

/**
 * @brief Traits are traits classes to determine the type of a derivative of another type.
 *
 * For geometric objects the "geometric" derivative is chosen.  For example the derivative of a
 * Rotation matrix is NOT a 3x3 matrix containing the derivative of the elements of a rotation
 * matrix.  The derivative of the rotation matrix is a Vector corresponding the rotational velocity.
 * Mostly used in template classes and routines to derive a correct type when needed.
 *
 * You can see this as a compile-time lookuptable to find the type of the derivative.
 */
template <typename T>
struct Traits
{
  typedef T valueType;
  typedef T derivType;
};

template <>
struct Traits<float>
{
  typedef float valueType;
  typedef float derivType;
};

template <>
struct Traits<double>
{
  typedef double valueType;
  typedef double derivType;
};

template <>
struct Traits<Vector>
{
  typedef Vector valueType;
  typedef Vector derivType;
};

template <>
struct Traits<Rotation>
{
  typedef Rotation valueType;
  typedef Vector derivType;
};

template <>
struct Traits<Frame>
{
  typedef Frame valueType;
  typedef Twist derivType;
};

template <>
struct Traits<Twist>
{
  typedef Twist valueType;
  typedef Twist derivType;
};

template <>
struct Traits<Wrench>
{
  typedef Wrench valueType;
  typedef Wrench derivType;
};

template <>
struct Traits<doubleVel>
{
  typedef double valueType;
  typedef doubleVel derivType;
};

template <>
struct Traits<VectorVel>
{
  typedef Vector valueType;
  typedef VectorVel derivType;
};

template <>
struct Traits<RotationVel>
{
  typedef Rotation valueType;
  typedef VectorVel derivType;
};
}  // namespace KDL
