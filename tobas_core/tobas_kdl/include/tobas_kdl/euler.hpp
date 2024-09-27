#pragma once

#include <tobas_std_tools/geometry.hpp>

#include "./vector.hpp"
#include "./rotation.hpp"
#include "./quaternion.hpp"
#include "./angle_axis.hpp"

namespace kdl
{
/**
 * @brief ZYXオイラー角．
 */
class Euler
{
public:
  double roll, pitch, yaw;

  inline explicit Euler();
  inline explicit Euler(const double& _roll, const double& _pitch, const double& _yaw);
  inline explicit Euler(const Vector& rpy);
  inline explicit Euler(const Rotation& rot);

  inline static Euler Zero();

  inline void setZero();

  inline Vector toVector() const;
  inline Rotation toRotation() const;
  inline Quaternion toQuaternion() const;

  /* 2つの回転の差を等価角軸ベクトルとして返す．O_AngleAxis_AB = O_Rot_B - O_Rot_A． */
  inline AngleAxis operator-(const Euler& rhs) const;

  /* 3次元ベクトルを回転させる． */
  inline Vector operator*(const Vector& v) const;

  /* 3次元ベクトルを逆回転させる． */
  inline Vector inverse(const Vector& v) const;

  bool isFinite() const;

  friend std::ostream& operator<<(std::ostream& os, const Euler& arg);
};

inline Euler::Euler() : roll(0), pitch(0), yaw(0)
{
}

inline Euler::Euler(const double& _roll, const double& _pitch, const double& _yaw)
  : roll(_roll), pitch(_pitch), yaw(_yaw)
{
}

inline Euler::Euler(const Vector& rpy) : roll(rpy.x()), pitch(rpy.y()), yaw(rpy.z())
{
}

inline Euler::Euler(const Rotation& rot)
{
  rot.getRPY(roll, pitch, yaw);
}

inline Euler Euler::Zero()
{
  return Euler(0., 0., 0.);
}

inline void Euler::setZero()
{
  roll = 0.;
  pitch = 0.;
  yaw = 0.;
}

inline Vector Euler::toVector() const
{
  return Vector(roll, pitch, yaw);
}

inline Rotation Euler::toRotation() const
{
  return Rotation::RPY(roll, pitch, yaw);
}

inline Quaternion Euler::toQuaternion() const
{
  Quaternion res;
  tobas_std::eulerToQuaternion(roll, pitch, yaw, res.x, res.y, res.z, res.w);
  return res;
}

inline AngleAxis Euler::operator-(const Euler& rhs) const
{
  return AngleAxis((toRotation() * rhs.toRotation().inverse()).getRot());
}

inline Vector Euler::operator*(const Vector& v) const
{
  return Rotation::RPY(roll, pitch, yaw) * v;
}

inline Vector Euler::inverse(const Vector& v) const
{
  return Rotation::RPY(roll, pitch, yaw).inverse(v);
}
}  // namespace kdl
