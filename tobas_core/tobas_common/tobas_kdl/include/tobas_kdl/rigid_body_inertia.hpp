// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include "./frames.hpp"
#include "./rotational_inertia.hpp"
#include "./segment_jacobian.hpp"

namespace tobas::kdl
{
/**
 * 6D Inertia of a rigid body.
 *
 * The inertia is defined in a certain reference point and a certain reference base.
 * The reference point does not have to coincide with the origin of the reference frame.
 */
class RigidBodyInertia
{
public:
  /**
   * This constructor creates a cartesian space inertia matrix, the arguments are the mass,
   * the vector from the reference point to cog and the rotational nertia in the cog.
   */
  explicit RigidBodyInertia(
    double _m = 0.0,
    const Vector& _oc = Vector::Zero(),
    const RotationalInertia& _ic = RotationalInertia::Zero());

  /** Creates an inertia with zero mass, and zero RotationalInertia */
  static inline RigidBodyInertia Zero();

  /** Verify that the rotation matrix belongs to SO(3). */
  std::expected<void, std::string> validate() const;

  /** Get the mass of the rigid body. */
  inline const double& getMass() const;

  /** Get the spatial momentum of the rigid body. */
  inline const Vector& getSpatialMomentum() const;

  /** Get the rotational inertia expressed in the reference frame (not the cog). */
  inline const RotationalInertia& getRotationalInertia() const;

  /** Get the center of gravity of the rigid body. */
  inline Vector getCOG() const;

  /** Get the rotational inertia expressed in the center of gravity. */
  inline RotationalInertia getRotationalInertiaCoG() const;

  /**
   * Reference point change with v the vector from the old to the new point
   * expressed in the current reference frame.
   */
  RigidBodyInertia refPoint(const Vector& _p) const;

  inline RigidBodyInertia operator+(const RigidBodyInertia& _rhs) const;
  inline RigidBodyInertia& operator+=(const RigidBodyInertia& _rhs);

  inline Impulse operator*(const Twist& _rhs) const;
  inline Wrench operator*(const Accel& _rhs) const;
  inline SegmentInertia operator*(const SegmentJacobian& _rhs) const;

  inline friend RigidBodyInertia operator*(double _a, const RigidBodyInertia& _i);
  friend RigidBodyInertia operator*(const Frame& _t, const RigidBodyInertia& _i);
  inline friend RigidBodyInertia operator*(const Rotation& _r, const RigidBodyInertia& _i);

  friend std::ostream& operator<<(std::ostream& _os, const RigidBodyInertia& _arg);

private:
  double m_;             ///< [kg]
  Vector h_;             ///< [kg m]
  RotationalInertia I_;  ///< [kg m^2]

  inline explicit RigidBodyInertia(double _m, const Vector& _h, const RotationalInertia& _i, bool _mhi);
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

inline RigidBodyInertia RigidBodyInertia::refPoint(const Vector& _p) const
{
  const auto hmr = h_ - m_ * _p;
  const auto& r_eig = _p.data;
  const auto& h_eig = h_.data;
  const auto& hmr_eig = hmr.data;

  Eigen::Matrix3d rcrosshcross = h_eig * r_eig.transpose();
  rcrosshcross.diagonal().array() -= r_eig.dot(h_eig);

  Eigen::Matrix3d hmrcrossrcross = r_eig * hmr_eig.transpose();
  hmrcrossrcross.diagonal().array() -= hmr_eig.dot(r_eig);

  const RotationalInertia Ib(I_.data + rcrosshcross + hmrcrossrcross);
  return RigidBodyInertia(m_, hmr, Ib, true);
}

inline RigidBodyInertia RigidBodyInertia::operator+(const RigidBodyInertia& _rhs) const
{
  return RigidBodyInertia(m_ + _rhs.m_, h_ + _rhs.h_, I_ + _rhs.I_, true);
}

inline RigidBodyInertia& RigidBodyInertia::operator+=(const RigidBodyInertia& _rhs)
{
  m_ += _rhs.m_;
  h_ += _rhs.h_;
  I_ += _rhs.I_;
  return *this;
}

inline Impulse RigidBodyInertia::operator*(const Twist& _rhs) const
{
  return Impulse(m_ * _rhs.vel - h_ * _rhs.rot, I_ * _rhs.rot + h_ * _rhs.vel);
}

inline Wrench RigidBodyInertia::operator*(const Accel& _rhs) const
{
  return Wrench(m_ * _rhs.linear - h_ * _rhs.angular, I_ * _rhs.angular + h_ * _rhs.linear);
}

inline SegmentInertia RigidBodyInertia::operator*(const SegmentJacobian& _rhs) const
{
  return SegmentInertia(m_ * _rhs.linear - h_ * _rhs.angular, I_ * _rhs.angular + h_ * _rhs.linear);
}

inline RigidBodyInertia operator*(double _a, const RigidBodyInertia& _i)
{
  return RigidBodyInertia(_a * _i.m_, _a * _i.h_, _a * _i.I_, true);
}

inline RigidBodyInertia operator*(const Frame& _t, const RigidBodyInertia& _i)
{
  const auto X = _t.inverse();
  const auto hmr = _i.h_ - _i.m_ * X.p;
  const auto& R = _t.M.data;
  const auto& r_eig = X.p.data;
  const auto& h_eig = _i.h_.data;
  const auto& hmr_eig = hmr.data;

  Eigen::Matrix3d rcrosshcross = h_eig * r_eig.transpose();
  rcrosshcross.diagonal().array() -= r_eig.dot(h_eig);

  Eigen::Matrix3d hmrcrossrcross = r_eig * hmr_eig.transpose();
  hmrcrossrcross.diagonal().array() -= hmr_eig.dot(r_eig);

  const RotationalInertia Ib(R * (_i.I_.data + rcrosshcross + hmrcrossrcross) * R.transpose());
  return RigidBodyInertia(_i.m_, _t.M * hmr, Ib, true);
}

inline RigidBodyInertia operator*(const Rotation& _m, const RigidBodyInertia& _i)
{
  return RigidBodyInertia(_i.m_, _m * _i.h_, _m * _i.I_, true);
}

inline RigidBodyInertia::RigidBodyInertia(double _m, const Vector& _h, const RotationalInertia& _i, bool)
  : m_(_m), h_(_h), I_(_i)
{
}
}  // namespace tobas::kdl
