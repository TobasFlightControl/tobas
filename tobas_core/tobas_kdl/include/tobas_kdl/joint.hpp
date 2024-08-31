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

  explicit Joint();

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
  Frame pose(const double& q) const;

  /**
   * Request the resulting 6D-velocity with a joint velocity qd.
   *
   * @param qd the 1D joint velocity
   *
   * @return the resulting 6D-velocity
   */
  Twist twist(const double& qd) const;

  /**
   * Request the resulting 6D-acceleration with a joint acceleration qdd.
   *
   * @param qdd the 1D joint acceleration
   *
   * @return the resulting 6D-acceleration
   */
  Accel accel(const double& qdd) const;

  /**
   * Request the jacobian for this joint.
   */
  SegmentJacobian jacobian() const;

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

inline const Vector& Joint::axis() const
{
  return axis_;
}

inline void Joint::axis(const Vector& axis)
{
  assert(axis.norm() > 0);
  axis_ = axis.normalized();
}
}  // end of namespace kdl
