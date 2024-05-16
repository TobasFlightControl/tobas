#pragma once

#include "./frames.hpp"
#include "./rotationalinertia.hpp"
#include "./segmentjacobian.hpp"

namespace tobas_kdl
{
/**
 *	\brief 6D Inertia of a rigid body
 *
 *	The inertia is defined in a certain reference point and a certain reference base.
 *	The reference point does not have to coincide with the origin of the reference frame.
 */
class RigidBodyInertia
{
public:
  /**
   * This constructor creates a cartesian space inertia matrix,
   * the arguments are the mass, the vector from the reference point to cog and the rotational
   * inertia in the cog.
   */
  explicit RigidBodyInertia(
    double m = 0,
    const Vector& oc = Vector::Zero(),
    const RotationalInertia& Ic = RotationalInertia::Zero());

  /* Creates an inertia with zero mass, and zero RotationalInertia */
  inline static RigidBodyInertia Zero();

  /* Get the mass of the rigid body */
  inline const double& getMass() const;

  /* Get the spatial momentum of the rigid body */
  inline const Vector& getSpatialMomentum() const;

  /* Get the rotational inertia expressed in the reference frame (not the cog) */
  inline const RotationalInertia& getRotationalInertia() const;

  /* Get the center of gravity of the rigid body */
  inline Vector getCOG() const;

  /* Get the rotational inertia expressed in the center of gravity. */
  inline RotationalInertia getRotationalInertiaCoG() const;

  /**
   * Reference point change with v the vector from the old to
   * the new point expressed in the current reference frame
   */
  RigidBodyInertia refPoint(const Vector& p) const;

  inline RigidBodyInertia operator+(const RigidBodyInertia& rhs) const;
  inline Impulse operator*(const Twist& rhs) const;
  inline Wrench operator*(const Accel& rhs) const;
  inline SegmentInertia operator*(const SegmentJacobian& rhs) const;

  inline friend RigidBodyInertia operator*(double a, const RigidBodyInertia& I);

  friend RigidBodyInertia operator*(const Frame& T, const RigidBodyInertia& I);
  friend RigidBodyInertia operator*(const Rotation& R, const RigidBodyInertia& I);

private:
  double m_;             // [kg]
  Vector h_;             // [kg m]
  RotationalInertia I_;  // [kg m^2]

  inline explicit RigidBodyInertia(double m, const Vector& h, const RotationalInertia& I, bool mhi);
};

inline RigidBodyInertia RigidBodyInertia::Zero()
{
  return RigidBodyInertia(0, Vector::Zero(), RotationalInertia::Zero());
};

inline const double& RigidBodyInertia::getMass() const
{
  return m_;
};

inline const Vector& RigidBodyInertia::getSpatialMomentum() const
{
  return h_;
}

inline const RotationalInertia& RigidBodyInertia::getRotationalInertia() const
{
  return I_;
};

inline Vector RigidBodyInertia::getCOG() const
{
  return m_ == 0 ? Vector::Zero() : h_ / m_;
};

inline RotationalInertia RigidBodyInertia::getRotationalInertiaCoG() const
{
  return refPoint(getCOG()).getRotationalInertia();
}

inline RigidBodyInertia RigidBodyInertia::operator+(const RigidBodyInertia& rhs) const
{
  return RigidBodyInertia(m_ + rhs.m_, h_ + rhs.h_, I_ + rhs.I_, true);
}

inline Impulse RigidBodyInertia::operator*(const Twist& rhs) const
{
  return Impulse(m_ * rhs.vel - h_ * rhs.rot, I_ * rhs.rot + h_ * rhs.vel);
}

inline Wrench RigidBodyInertia::operator*(const Accel& rhs) const
{
  return Wrench(m_ * rhs.linear - h_ * rhs.angular, I_ * rhs.angular + h_ * rhs.linear);
}

inline SegmentInertia RigidBodyInertia::operator*(const SegmentJacobian& rhs) const
{
  return SegmentInertia(m_ * rhs.linear - h_ * rhs.angular, I_ * rhs.angular + h_ * rhs.linear);
}

inline RigidBodyInertia operator*(double a, const RigidBodyInertia& I)
{
  return RigidBodyInertia(a * I.m_, a * I.h_, a * I.I_, true);
}

inline RigidBodyInertia::RigidBodyInertia(
  double m,
  const Vector& h,
  const RotationalInertia& I,
  bool)
  : m_(m), h_(h), I_(I)
{
}
}  // namespace tobas_kdl
