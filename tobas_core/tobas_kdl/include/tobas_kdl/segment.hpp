#pragma once

#include "./frames.hpp"
#include "./rigidbodyinertia.hpp"
#include "./joint.hpp"
#include "./segmentjacobian.hpp"

namespace kdl
{
/**
 * @brief This class encapsulates a simple segment, that is a "rigid body" (i.e., a frame and a rigid body inertia)
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
   * @param name Name of the segment
   * @param joint Joint of the segment
   * @param f_tip Frame from the end of the joint to the tip of the segment
   * @param I Rigid body inertia of the segment
   */
  inline explicit Segment(
    const std::string& name = "",
    const Joint& joint = Joint(),
    const Frame& f_tip = Frame::Identity(),
    const RigidBodyInertia& I = RigidBodyInertia::Zero());

  /* Request the pose of the segment wrt. the parent frame. */
  inline Frame pose(double q) const;

  /* Request the 6D-velocity of the tip of the segment wrt. the parent frame. */
  inline Twist twist(double q, double qd) const;

  /* Request the jacobian for the joint of this segment wrt. the parent frame. */
  inline SegmentJacobian jacobian(double q) const;

  /* Request the name of the segment. */
  inline const std::string& name() const;

  /* Request the joint of the segment. */
  inline const Joint& joint() const;

  /**
   * @brief Compute the first derivative of the rotation matrix from the parent segment to the tip of this segment
   * with respect to the joint position.
   */
  inline Eigen::Matrix3d rotGrad(double q) const;

  /**
   * @brief Compute the second derivative of the rotation matrix from the parent segment to the tip of this segment
   * with respect to the joint position.
   */
  inline Eigen::Matrix3d rotGrad2(double q) const;

  /* Request the tip frame of the segment. */
  inline Frame frame() const;

  /* Request the inertia of the segment. */
  inline const RigidBodyInertia& inertia() const;

  inline friend std::ostream& operator<<(std::ostream& os, const Segment& arg);

private:
  std::string name_;
  Joint joint_;
  Frame f_tip_;
  RigidBodyInertia I_;
};

inline Segment::Segment(const std::string& name, const Joint& joint, const Frame& f_tip, const RigidBodyInertia& I)
  : name_(name), joint_(joint), f_tip_(joint.pose(0).inverse() * f_tip), I_(I)
{
}

inline Frame Segment::pose(double q) const
{
  return joint_.pose(q) * f_tip_;
}

inline Twist Segment::twist(double q, double qd) const
{
  return joint_.twist(qd).refPoint(joint_.pose(q).M * f_tip_.p);
}

inline SegmentJacobian Segment::jacobian(double q) const
{
  return joint_.jacobian().refPoint(joint_.pose(q).M * f_tip_.p);
}

inline Eigen::Matrix3d Segment::rotGrad(double q) const
{
  return joint_.rotGrad(q) * f_tip_.M.data;
}

inline Eigen::Matrix3d Segment::rotGrad2(double q) const
{
  return joint_.rotGrad2(q) * f_tip_.M.data;
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
  return joint_.pose(0) * f_tip_;
}

inline const RigidBodyInertia& Segment::inertia() const
{
  return I_;
}

inline std::ostream& operator<<(std::ostream& os, const Segment& arg)
{
  os << "Name: " << arg.name_ << std::endl;
  os << "Joint:\n" << arg.joint_ << std::endl;
  os << "Frame:\n" << arg.f_tip_ << std::endl;
  os << "Inertia:\n" << arg.I_;
  return os;
}
}  // end of namespace kdl
