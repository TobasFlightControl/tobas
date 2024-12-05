#pragma once

#include "./frames.hpp"
#include "./rigidbodyinertia.hpp"
#include "./joint.hpp"
#include "./segmentjacobian.hpp"

namespace kdl
{
/**
 * \brief This class encapsulates a simple segment, that is a "rigid body" (i.e., a frame and a rigid body inertia)
 * with a joint and with "handles", root and tip to connect to other segments.
 *
 * A simple segment is described by the following properties :
 *      - Joint
 *      - Rigid Body Inertia: of the rigid body part of the Segment
 *      - Offset from the end of the joint to the tip of the segment:
 *        the joint is located at the root of the segment.
 */
class Segment
{
public:
  /**
   * Constructor of the segment.
   *
   * @param name name of the segment, default: ""
   * @param joint joint of the segment, default: Joint(Joint::FIXED)
   * @param f_tip frame from the end of the joint to the tip of the segment, default: Frame::Identity()
   * @param I rigid body inertia of the segment, default: Inertia::Zero()
   */
  inline explicit Segment(
    const std::string& name = "",
    const Joint& joint = Joint(),
    const Frame& f_tip = Frame::Identity(),
    const RigidBodyInertia& I = RigidBodyInertia::Zero());

  /**
   * Request the pose of the segment, given the joint position q.
   *
   * @param q 1D position of the joint
   *
   * @return pose from the root to the tip of the segment
   */
  inline Frame pose(const double& q) const;

  /**
   * Request the 6D-velocity of the tip of the segment, given the joint position q and the joint velocity qd.
   *
   * @param q 1D position of the joint
   * @param qd 1D velocity of the joint
   *
   * @return 6D-velocity of the tip of the segment,
   * expressed in the base-frame of the segment(root) and with the tip of the segment as reference point.
   */
  inline Twist twist(const double& q, const double& qd) const;

  /**
   * Request the jacobian for the joint of this segment, given the joint position q.
   *
   * @param q 1D position of the joint
   *
   * @return cartesian jacobian, expressed in the base-frame of the segment(root)
   * and with the tip of the segment as reference point.
   */
  inline SegmentJacobian jacobian(const double& q) const;

  /* Request the name of the segment. */
  inline const std::string& name() const;

  /** Request the joint of the segment. */
  inline const Joint& joint() const;

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

inline Frame Segment::pose(const double& q) const
{
  return joint_.pose(q) * f_tip_;
}

inline Twist Segment::twist(const double& q, const double& qd) const
{
  return joint_.twist(qd).refPoint(joint_.pose(q).M * f_tip_.p);
}

inline SegmentJacobian Segment::jacobian(const double& q) const
{
  return joint_.jacobian().refPoint(joint_.pose(q).M * f_tip_.p);
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
