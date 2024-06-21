#pragma once

#include "./frames.hpp"

namespace kdl
{
class RotationalInertia
{
public:
  Eigen::Matrix3d data;

  explicit RotationalInertia(
    double Ixx = 0,
    double Iyy = 0,
    double Izz = 0,
    double Ixy = 0,
    double Ixz = 0,
    double Iyz = 0);
  explicit RotationalInertia(const Eigen::Matrix3d& data);

  inline static RotationalInertia Zero();

  inline double ixx() const;
  inline double iyy() const;
  inline double izz() const;
  inline double ixy() const;
  inline double ixz() const;
  inline double iyz() const;

  inline double trace() const;

  inline RotationalInertia operator+(const RotationalInertia& rhs) const;
  inline RotationalInertia& operator+=(const RotationalInertia& rhs);

  /* Compute the angular momentum resulting from a rotational velocity omega. */
  inline Vector operator*(const Vector& omega) const;

  inline friend RotationalInertia operator*(double a, const RotationalInertia& I);
  /* Compute Ia = Rab Ib Rab^T. */
  inline friend RotationalInertia operator*(const Rotation& R_a_b, const RotationalInertia& I_b);
};

inline RotationalInertia RotationalInertia::Zero()
{
  return RotationalInertia(0, 0, 0, 0, 0, 0);
}

inline double RotationalInertia::ixx() const
{
  return data(0, 0);
}

inline double RotationalInertia::iyy() const
{
  return data(1, 1);
}

inline double RotationalInertia::izz() const
{
  return data(2, 2);
}

inline double RotationalInertia::ixy() const
{
  return data(0, 1);
}

inline double RotationalInertia::ixz() const
{
  return data(0, 2);
}

inline double RotationalInertia::iyz() const
{
  return data(1, 2);
}

inline double RotationalInertia::trace() const
{
  return data.trace();
}

inline RotationalInertia RotationalInertia::operator+(const RotationalInertia& rhs) const
{
  return RotationalInertia(data + rhs.data);
}

inline RotationalInertia& RotationalInertia::operator+=(const RotationalInertia& rhs)
{
  data += rhs.data;
  return *this;
}

inline Vector RotationalInertia::operator*(const Vector& omega) const
{
  return Vector(data * omega.data);
}

inline RotationalInertia operator*(double a, const RotationalInertia& I)
{
  return RotationalInertia(a * I.data);
}

inline RotationalInertia operator*(const Rotation& R_a_b, const RotationalInertia& I_b)
{
  return RotationalInertia(R_a_b.data * I_b.data * R_a_b.data.transpose());
}
}  // namespace kdl
