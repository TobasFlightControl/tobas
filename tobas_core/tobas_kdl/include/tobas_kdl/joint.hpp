#pragma once

#include <string>

#include <tobas_eigen_tools/geometry.hpp>

#include "./frames.hpp"
#include "./segment_jacobian.hpp"

namespace kdl
{
/**
 * @brief This class encapsulates a simple joint, that is with one
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
  double lower_limit = -INFINITY;
  double upper_limit = INFINITY;
  double max_effort = INFINITY;
  double max_velocity = INFINITY;

  inline explicit Joint();

  /* Get the normalized joint axis wrt. the parent frame. */
  inline const Vector& axis() const;

  /* Set joint axis. */
  inline void axis(const Vector& axis);

  /* Request the 6D-pose of the end of the joint wrt. the parent frame. */
  inline Frame pose(double q) const;

  /* Request the resulting 6D-velocity of the end of the joint wrt. the parent frame. */
  inline Twist twist(double qd) const;

  /* Request the resulting 6D-acceleration of the end of the joint wrt. the parent frame. */
  inline Accel accel(double qdd) const;

  /* Request the jacobian for this joint wrt. the parent frame. */
  inline SegmentJacobian jacobian() const;

  /* Compute the first derivative of the rotation matrix with respect to the joint position. */
  inline Eigen::Matrix3d rotGrad(double q) const;

  /* Compute the second derivative of the rotation matrix with respect to the joint position. */
  inline Eigen::Matrix3d rotGrad2(double q) const;

  static inline const char* typeToText(joint_type_t type);

  inline friend std::ostream& operator<<(std::ostream& os, const Joint& arg);

private:
  Vector axis_ = Vector::UnitZ();  // The normalized joint axis wrt. the parent frame.
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

inline Frame Joint::pose(double q) const
{
  switch (type) {
    case ROTATION:
      return Frame(Rotation::Rot(axis_, q), origin);
    case TRANSLATION:
      return Frame(origin + (axis_ * q));
    case FIXED:
      return Frame::Identity();
    default:
      throw;
  }
}

inline Twist Joint::twist(double qd) const
{
  switch (type) {
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

inline Accel Joint::accel(double qdd) const
{
  switch (type) {
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
  switch (type) {
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

inline Eigen::Matrix3d Joint::rotGrad(double q) const
{
  if (type == ROTATION) {
    return eigen::skew(axis_.data) * Rotation::Rot(axis_, q).data;
  }
  else {
    return Eigen::Matrix3d::Zero();
  }
}

inline Eigen::Matrix3d Joint::rotGrad2(double q) const
{
  if (type == ROTATION) {
    return eigen::skew2(axis_.data) * Rotation::Rot(axis_, q).data;
  }
  else {
    return Eigen::Matrix3d::Zero();
  }
}

inline const char* Joint::typeToText(joint_type_t type)
{
  switch (type) {
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
