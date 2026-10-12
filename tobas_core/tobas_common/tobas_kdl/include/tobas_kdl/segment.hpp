// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include "./frames.hpp"
#include "./joint.hpp"
#include "./rigid_body_inertia.hpp"
#include "./segment_jacobian.hpp"

namespace tobas
{
namespace kdl
{
/**
 * This class encapsulates a simple segment, that is a "rigid body" (i.e., a frame and a rigid body inertia)
 * with a joint and with "handles", root and tip to connect to other segments.
 *
 * A simple segment is described by the following properties:
 *  - Joint
 *  - Rigid Body Inertia:
 *      of the rigid body part of the segment
 *  - Offset from the end of the joint to the tip of the segment:
 *      the joint is located at the root of the segment
 */
class Segment
{
public:
  /**
   * Constructor of the segment.
   *
   * @param _name Name of the segment
   * @param _joint Joint of the segment
   * @param _f_tip Frame from the end of the joint to the tip of the segment
   * @param _i Rigid body inertia of the segment
   */
  inline explicit Segment(
    const std::string& _name = "",
    const Joint& _joint = Joint(),
    const Frame& _f_tip = Frame::Identity(),
    const RigidBodyInertia& _i = RigidBodyInertia::Zero());

  /** Check validity (except for the root segment). */
  std::expected<void, std::string> validate() const;

  /** Request the pose of the segment wrt. the parent frame. */
  inline Frame pose(double _q) const;

  /** Request the 6D-velocity of the tip of the segment wrt. the parent frame. */
  inline Twist twist(double _q, double _qd) const;

  /** Request the jacobian for the joint of this segment wrt. the parent frame. */
  inline SegmentJacobian jacobian(double _q) const;

  /** Request the name of the segment. */
  inline const std::string& name() const;

  /** Request the joint of the segment. */
  inline const Joint& joint() const;

  /**
   * Compute the first derivative of the rotation matrix from the parent segment to the tip of this segment
   * with respect to the joint position.
   */
  inline Eigen::Matrix3d rotGrad(double _q) const;

  /**
   * Compute the second derivative of the rotation matrix from the parent segment to the tip of this segment
   * with respect to the joint position.
   */
  inline Eigen::Matrix3d rotGrad2(double _q) const;

  /** Request the frame from the end of the joint (= drive shaft) to the tip of the segment. */
  inline Frame frame() const;

  /** Request the inertia of the segment. */
  inline const RigidBodyInertia& inertia() const;

  friend std::ostream& operator<<(std::ostream& _os, const Segment& _arg);

private:
  std::string name_;
  Joint joint_;
  Frame f_tip_;
  RigidBodyInertia I_;
};

inline Segment::Segment(const std::string& _name, const Joint& _joint, const Frame& _f_tip, const RigidBodyInertia& _i)
  : name_(_name), joint_(_joint), f_tip_(_joint.pose(0.0).inverse() * _f_tip), I_(_i)
{
}

inline Frame Segment::pose(double _q) const
{
  return joint_.pose(_q) * f_tip_;
}

inline Twist Segment::twist(double _q, double _qd) const
{
  return joint_.twist(_qd).refPoint(joint_.pose(_q).M * f_tip_.p);
}

inline SegmentJacobian Segment::jacobian(double _q) const
{
  return joint_.jacobian().refPoint(joint_.pose(_q).M * f_tip_.p);
}

inline Eigen::Matrix3d Segment::rotGrad(double _q) const
{
  return joint_.rotGrad(_q) * f_tip_.M.data;
}

inline Eigen::Matrix3d Segment::rotGrad2(double _q) const
{
  return joint_.rotGrad2(_q) * f_tip_.M.data;
}

inline const std::string& Segment::name() const
{
  return name_;
}

inline const Joint& Segment::joint() const
{
  return joint_;
}

inline Frame Segment::frame() const
{
  return joint_.pose(0.0) * f_tip_;
}

inline const RigidBodyInertia& Segment::inertia() const
{
  return I_;
}
}  // namespace kdl
}  // namespace tobas
