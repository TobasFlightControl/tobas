#pragma once

#include "./frames.hpp"
#include "./rigidbodyinertia.hpp"
#include "./joint.hpp"
#include "./segmentjacobian.hpp"

namespace kdl
{
/**
 * \brief This class encapsulates a simple segment, that is a "rigid
 * body" (i.e., a frame and a rigid body inertia) with a joint and with
 * "handles", root and tip to connect to other segments.
 *
 * A simple segment is described by the following properties :
 *      - Joint
 *      - Rigid Body Inertia: of the rigid body part of the Segment
 *      - Offset from the end of the joint to the tip of the segment:
 *        the joint is located at the root of the segment.
 */
class Segment
{
  friend class Chain;

public:
  /**
   * Constructor of the segment
   *
   * @param name name of the segment
   * @param joint joint of the segment, default:
   * Joint(Joint::Fixed)
   * @param f_tip frame from the end of the joint to the tip of
   * the segment, default: Frame::Identity()
   * @param M rigid body inertia of the segment, default: Inertia::Zero()
   */
  inline explicit Segment(
    const std::string& name,
    const Joint& joint = Joint(),
    const Frame& f_tip = Frame::Identity(),
    const RigidBodyInertia& I = RigidBodyInertia::Zero());

  /**
   * Constructor of the segment
   *
   * @param joint joint of the segment, default:
   * Joint(Joint::Fixed)
   * @param f_tip frame from the end of the joint to the tip of
   * the segment, default: Frame::Identity()
   * @param M rigid body inertia of the segment, default: Inertia::Zero()
   */
  inline explicit Segment(
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
   * Request the 6D-velocity of the tip of the segment, given
   * the joint position q and the joint velocity qd.
   *
   * @param q 1D position of the joint
   * @param qd 1D velocity of the joint
   *
   * @return 6D-velocity of the tip of the segment, expressed
   * in the base-frame of the segment(root) and with the tip of the segment as reference point.
   */
  inline Twist twist(const double& q, const double& qd) const;

  /**
   * Request the jacobian for the joint of this segment, given the joint position q.
   *
   * @param q 1D position of the joint
   *
   * @return cartesian jacobian, expressed
   * in the base-frame of the segment(root) and with the tip of the segment as reference point.
   */
  inline SegmentJacobian jacobian(const double& q) const;

  /**
   * Request the name of the segment
   *
   * @return const reference to the name of the segment
   */
  inline const std::string& name() const;

  /**
   * Request the joint of the segment
   *
   * @return const reference to the joint of the segment
   */
  inline const Joint& getJoint() const;

  /**
   * Request the inertia of the segment
   *
   * @return const reference to the inertia of the segment
   */
  inline const RigidBodyInertia& getInertia() const;

  /**
   * Request the inertia of the segment
   *
   * @return const reference to the inertia of the segment
   */
  inline void setInertia(const RigidBodyInertia& Iin);

  /**
   * Request the pose from the joint end to the tip of the segment.
   *
   * @return the original parent end - segment end pose.
   */
  inline Frame getFrameToTip() const;

  /**
   * Set the pose from the joint end to the tip of the
   * segment.
   *
   * @param f_tip_new pose from the joint end to the tip of the segment
   */
  inline void setFrameToTip(const Frame& f_tip_new);

  /**
   * Request the pose from the end of the joint to the tip of the segment
   * at joint position 0.
   *
   * @return const reference to the pose from the end of the joint to the tip of the segment
   * at joint position 0
   */
  inline const Frame& getFrameToTipZero() const;

private:
  std::string name_;
  Joint joint_;
  RigidBodyInertia I_;
  Frame f_tip_;
};

inline Segment::Segment(const std::string& name, const Joint& joint, const Frame& f_tip, const RigidBodyInertia& I)
  : name_(name), joint_(joint), I_(I), f_tip_(joint.pose(0).inverse() * f_tip)
{
}

inline Segment::Segment(const Joint& joint, const Frame& f_tip, const RigidBodyInertia& I)
  : name_("NoName"), joint_(joint), I_(I), f_tip_(joint.pose(0).inverse() * f_tip)
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

inline const Joint& Segment::getJoint() const
{
  return joint_;
}

inline const RigidBodyInertia& Segment::getInertia() const
{
  return I_;
}

inline void Segment::setInertia(const RigidBodyInertia& Iin)
{
  I_ = Iin;
}

inline Frame Segment::getFrameToTip() const
{
  return joint_.pose(0) * f_tip_;
}

inline void Segment::setFrameToTip(const Frame& f_tip_new)
{
  f_tip_ = joint_.pose(0).inverse() * f_tip_new;
}

inline const Frame& Segment::getFrameToTipZero() const
{
  return f_tip_;
}
}  // end of namespace kdl
