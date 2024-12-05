#pragma once

#include <string>
#include <exception>

#include "./frames.hpp"
#include "./segmentjacobian.hpp"

namespace kdl
{
/**
 * \brief This class encapsulates a simple joint, that is with one
 * parameterized degree of freedom and with scalar dynamic properties.
 */
class Joint
{
public:
  enum JointType : uint8_t
  {
    RotAxis,
    TransAxis,
    Fixed,
  };

  std::string name = "";
  Joint::JointType type = Fixed;
  Vector origin = Vector::Zero();
  double damping = 0.;
  double friction = 0.;
  double lower_limit = std::numeric_limits<double>::lowest();
  double upper_limit = std::numeric_limits<double>::max();
  double max_effort = std::numeric_limits<double>::max();
  double max_velocity = std::numeric_limits<double>::max();

  inline explicit Joint();

  /* Get the normalized joint axis. */
  inline const Vector& axis() const;
  /* Set joint axis. */
  inline void axis(const Vector& axis);

  /**
   * Request the 6D-pose between the beginning and the end of
   * the joint at joint position q.
   *
   * @param q the 1D joint position
   *
   * @return the resulting 6D-pose
   */
  inline Frame pose(const double& q) const;

  /**
   * Request the resulting 6D-velocity with a joint velocity qd.
   *
   * @param qd the 1D joint velocity
   *
   * @return the resulting 6D-velocity
   */
  inline Twist twist(const double& qd) const;

  /**
   * Request the resulting 6D-acceleration with a joint acceleration qdd.
   *
   * @param qdd the 1D joint acceleration
   *
   * @return the resulting 6D-acceleration
   */
  inline Accel accel(const double& qdd) const;

  /**
   * Request the jacobian for this joint.
   */
  inline SegmentJacobian jacobian() const;

  friend std::ostream& operator<<(std::ostream& os, const Joint& arg);

private:
  class joint_type_exception : public std::exception
  {
    virtual const char* what() const throw()
    {
      return "Joint Type excption";
    }
  } joint_type_ex_;

  Vector axis_ = Vector::UnitZ();  // The axis of a movable joint must be normalized.
};

inline Joint::Joint()
{
}

inline const Vector& Joint::axis() const
{
  return axis_;
}

inline void Joint::axis(const Vector& axis)
{
  assert(axis.norm() > 0);
  axis_ = axis.normalized();
}

inline Frame Joint::pose(const double& q) const
{
  switch (type)
  {
    case RotAxis:
      return Frame(Rotation::Rot(axis_, q), origin);
    case TransAxis:
      return Frame(origin + (axis_ * (q)));
    case Fixed:
      return Frame::Identity();
    default:
      throw joint_type_ex_;
  }
}

inline Twist Joint::twist(const double& qd) const
{
  switch (type)
  {
    case RotAxis:
      return Twist(Vector::Zero(), axis_ * qd);
    case TransAxis:
      return Twist(axis_ * qd, Vector::Zero());
    case Fixed:
      return Twist::Zero();
    default:
      throw joint_type_ex_;
  }
}

inline Accel Joint::accel(const double& qdd) const
{
  switch (type)
  {
    case RotAxis:
      return Accel(Vector::Zero(), axis_ * qdd);
    case TransAxis:
      return Accel(axis_ * qdd, Vector::Zero());
    case Fixed:
      return Accel::Zero();
    default:
      throw joint_type_ex_;
  }
}

inline SegmentJacobian Joint::jacobian() const
{
  switch (type)
  {
    case RotAxis:
      return SegmentJacobian(Vector::Zero(), axis_);
    case TransAxis:
      return SegmentJacobian(axis_, Vector::Zero());
    case Fixed:
      return SegmentJacobian::Zero();
    default:
      throw joint_type_ex_;
  }
}
}  // end of namespace kdl
