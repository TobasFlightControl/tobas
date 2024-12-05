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
  enum joint_type_t : uint8_t
  {
    ROTATION,
    TRANSLATION,
    FIXED,
  };

  std::string name = "";
  Joint::joint_type_t type = FIXED;
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

  /* Request the 6D-pose between the beginning and the end of the joint at joint position q. */
  inline Frame pose(const double& q) const;

  /* Request the resulting 6D-velocity with a joint velocity qd. */
  inline Twist twist(const double& qd) const;

  /* Request the resulting 6D-acceleration with a joint acceleration qdd. */
  inline Accel accel(const double& qdd) const;

  /* Request the jacobian for this joint. */
  inline SegmentJacobian jacobian() const;

  inline static const char* typeToText(joint_type_t type);

  inline friend std::ostream& operator<<(std::ostream& os, const Joint& arg);

private:
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
    case ROTATION:
      return Frame(Rotation::Rot(axis_, q), origin);
    case TRANSLATION:
      return Frame(origin + (axis_ * (q)));
    case FIXED:
      return Frame::Identity();
    default:
      throw;
  }
}

inline Twist Joint::twist(const double& qd) const
{
  switch (type)
  {
    case ROTATION:
      return Twist(Vector::Zero(), axis_ * qd);
    case TRANSLATION:
      return Twist(axis_ * qd, Vector::Zero());
    case FIXED:
      return Twist::Zero();
    default:
      throw;
  }
}

inline Accel Joint::accel(const double& qdd) const
{
  switch (type)
  {
    case ROTATION:
      return Accel(Vector::Zero(), axis_ * qdd);
    case TRANSLATION:
      return Accel(axis_ * qdd, Vector::Zero());
    case FIXED:
      return Accel::Zero();
    default:
      throw;
  }
}

inline SegmentJacobian Joint::jacobian() const
{
  switch (type)
  {
    case ROTATION:
      return SegmentJacobian(Vector::Zero(), axis_);
    case TRANSLATION:
      return SegmentJacobian(axis_, Vector::Zero());
    case FIXED:
      return SegmentJacobian::Zero();
    default:
      throw;
  }
}

inline const char* Joint::typeToText(joint_type_t type)
{
  switch (type)
  {
    case ROTATION:
      return "Rotation";
    case TRANSLATION:
      return "Translation";
    case FIXED:
      return "Fixed";
    default:
      throw;
  }
}

inline std::ostream& operator<<(std::ostream& os, const Joint& arg)
{
  os << "Name: " << arg.name << std::endl;
  os << "Type: " << Joint::typeToText(arg.type) << std::endl;
  os << "Origin: " << arg.origin << std::endl;
  os << "Axis: " << arg.axis() << std::endl;
  os << "Damping: " << arg.damping << std::endl;
  os << "Friction: " << arg.friction << std::endl;
  os << "Lower Limit: " << arg.lower_limit << std::endl;
  os << "Upper Limit: " << arg.upper_limit << std::endl;
  os << "Max Effort: " << arg.max_effort << std::endl;
  os << "Max Velocity: " << arg.max_velocity;
  return os;
}
}  // end of namespace kdl
