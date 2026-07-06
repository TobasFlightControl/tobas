// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./accel.hpp"
#include "./impulse.hpp"
#include "./rotation.hpp"
#include "./twist.hpp"
#include "./vector.hpp"
#include "./wrench.hpp"

namespace tobas
{
namespace kdl
{
class Frame;
using FrameMap = std::map<std::string, Frame>;

/**
 * @brief represents a frame transformation in 3D space (rotation + translation)
 *
 * If V2 = Frame*V1 (V2 expressed in frame A, V1 expressed in frame B)
 * then V2 = Frame.M*V1+Frame.p
 *
 * Frame.M contains columns that represent the axes of frame B wrt frame A.
 * Frame.p contains the origin of frame B expressed in frame A.
 */
class Frame
{
public:
  Vector p;    // Origin of the frame
  Rotation M;  // Orientation of the frame

  inline explicit Frame(const Rotation& R, const Vector& V);
  // The rotation matrix defaults to identity
  inline explicit Frame(const Vector& V);
  // The position matrix defaults to zero
  inline explicit Frame(const Rotation& R);
  inline explicit Frame();

  // @return the identity transformation Frame(Rotation::Identity(),Vector::Zero()).
  static inline Frame Identity();

  /**
   * @brief Constructs a transformation matrix T_link(i-1)_link(i)
   * with the Denavit-Hartenberg convention as described in the Craigs book:
   * Craig, J. J., Introduction to ROBOTICS: Mechanics and Control, Addison-Wesley, isbn:0-201-10326-5, 1986.
   *
   * Note that the frame is a redundant way to express the information in the DH-convention.
   * Parameters in full : a(i-1), alpha(i-1), d(i), theta(i)
   *
   * Axis (i-1) is connected by link (i-1) to axis (i)
   * numbering axis (1) to axis (n) link (0) (immobile base) to link (n)
   *
   * Link length a(i-1): Length of the mutual perpendicular line (normal) between the 2 axes.
   * This normal runs from (i-1) to (i) axis.
   *
   * Link twist alpha(i-1): Construct plane perpendicular to the normal project axis(i-1) and axis(i)
   * into plane angle from (i-1) to (i) measured in the direction of the normal
   *
   * Link offset d(i): Signed distance between normal (i-1) to (i) and normal (i) to (i+1)
   * along axis (i) joint angle theta(i) signed angle
   * between normal (i-1) to (i) and normal (i) to (i+1) along axis (i)
   *
   * First and last joints : a(0)= a(n) = 0
   * alpha(0) = alpha(n) = 0
   *
   * PRISMATIC : theta(1) = 0 d(1) arbitrarily
   * REVOLUTE  : theta(1) arbitrarily d(1) = 0
   *
   * a(i-1)     = distance from Z(i-1) to Z(i) along X(i-1)
   * alpha(i-1) = angle between Z(i-1) to Z(i) along X(i-1)
   * d(i)       = distance from X(i-1) to X(i) along Z(i)
   * theta(i)   = angle between X(i-1) to X(i) along X(i)
   */
  static inline Frame DH_Craig1989(double a, double alpha, double d, double theta);

  /**
   * @brief Constructs a transformationmatrix T_link(i-1)_link(i) with the Denavit-Hartenberg convention
   * as described in the original publictation:
   * Denavit, J. and Hartenberg, R. S.,
   * A kinematic notation for lower-pair mechanisms based on matrices,
   * ASME Journal of Applied Mechanics, 23:215-221, 1955.
   */
  static inline Frame DH(double a, double alpha, double d, double theta);

  inline bool isValid(std::string& error_msg) const;

  inline Eigen::Matrix4d matrix() const;

  inline void setIdentity();
  inline void setNaN();

  /**
   * @brief Treats a frame as a 4x4 matrix and returns element i,j.
   * Access to elements 0..3,0..3, bounds are checked when NDEBUG is not set.
   */
  inline double operator()(int i, int j);
  /**
   * @brief Treats a frame as a 4x4 matrix and returns element i,j.
   * Access to elements 0..3,0..3, bounds are checked when NDEBUG is not set.
   */
  inline double operator()(int i, int j) const;

  /**
   * @brief The twist <t_this> is expressed wrt the current frame.
   * This frame is integrated into an updated frame with <samplefrequency>.
   * Very simple first order integration rule.
   */
  inline void integrate(const Twist& t_this, double frequency);

  /* Convert the `Frame` to a 6D `Twist`. */
  inline Twist toTwist() const;

  inline Frame inverse() const;
  inline Vector inverse(const Vector& arg) const;
  inline Twist inverse(const Twist& arg) const;
  inline Accel inverse(const Accel& arg) const;
  inline Wrench inverse(const Wrench& arg) const;

  inline Vector operator*(const Vector& arg) const;
  inline Wrench operator*(const Wrench& arg) const;
  inline SegmentInertia operator*(const SegmentInertia& arg) const;
  inline Twist operator*(const Twist& arg) const;
  inline Accel operator*(const Accel& arg) const;
  inline SegmentJacobian operator*(const SegmentJacobian& arg) const;
  inline Frame operator*(const Frame& rhs) const;

  /* Compute the difference of two frames wrt. the same frame. */
  inline Frame operator-(const Frame& rhs) const;

  inline friend std::ostream& operator<<(std::ostream& os, const Frame& arg);
};

inline Frame::Frame(const Rotation& R) : p(Vector::Zero()), M(R)
{
}

inline Frame::Frame(const Vector& V) : p(V), M(Rotation::Identity())
{
}

inline Frame::Frame(const Rotation& R, const Vector& V) : p(V), M(R)
{
}

inline Frame::Frame()
{
}

inline Frame Frame::Identity()
{
  return Frame(Rotation::Identity(), Vector::Zero());
}

inline Frame Frame::DH_Craig1989(double a, double alpha, double d, double theta)
{
  const auto ct = std::cos(theta);
  const auto st = std::sin(theta);
  const auto sa = std::sin(alpha);
  const auto ca = std::cos(alpha);
  return Frame(Rotation(ct, -st, 0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca), Vector(a, -sa * d, ca * d));
}

inline Frame Frame::DH(double a, double alpha, double d, double theta)
{
  const auto ct = std::cos(theta);
  const auto st = std::sin(theta);
  const auto sa = std::sin(alpha);
  const auto ca = std::cos(alpha);
  return Frame(Rotation(ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0, sa, ca), Vector(a * ct, a * st, d));
}

inline bool Frame::isValid(std::string& error_msg) const
{
  return M.isValid(error_msg);
}

inline Eigen::Matrix4d Frame::matrix() const
{
  Eigen::Matrix4d res;
  res.topLeftCorner<3, 3>() = M.data;
  res.topRightCorner<3, 1>() = p.data;
  res.bottomLeftCorner<1, 3>().setZero();
  res(3, 3) = 1;
  return res;
}

inline void Frame::setIdentity()
{
  p.setZero();
  M.setIdentity();
}

inline void Frame::setNaN()
{
  p.setNaN();
  M.setNaN();
}

inline double Frame::operator()(int i, int j)
{
  assert((0 <= i) && (i <= 3) && (0 <= j) && (j <= 3));
  if (i == 3) {
    if (j == 3) {
      return 1.0;
    }
    else {
      return 0.0;
    }
  }
  else {
    if (j == 3) {
      return p(i);
    }
    else {
      return M(i, j);
    }
  }
}

inline double Frame::operator()(int i, int j) const
{
  assert((0 <= i) && (i <= 3) && (0 <= j) && (j <= 3));
  if (i == 3) {
    if (j == 3) {
      return 1.0;
    }
    else {
      return 0.0;
    }
  }
  else {
    if (j == 3) {
      return p(i);
    }
    else {
      return M(i, j);
    }
  }
}

inline void Frame::integrate(const Twist& t_this, double sampling_freq)
{
  assert(sampling_freq > 0);

  const auto n = t_this.rot.norm() / sampling_freq;
  if (n < std::numeric_limits<double>::epsilon()) {
    p += M * (t_this.vel / sampling_freq);
  }
  else {
    (*this) = (*this) * Frame(Rotation::Rot(t_this.rot, n), t_this.vel / sampling_freq);
  }
}

inline Twist Frame::toTwist() const
{
  return Twist(p, M.getRot());
}

inline Frame Frame::inverse() const
{
  return Frame(M.inverse(), -M.inverse(p));
}

inline Vector Frame::inverse(const Vector& arg) const
{
  return M.inverse(arg - p);
}

inline Twist Frame::inverse(const Twist& arg) const
{
  Twist tmp;
  tmp.vel = M.inverse(arg.vel - p * arg.rot);
  tmp.rot = M.inverse(arg.rot);
  return tmp;
}

inline Accel Frame::inverse(const Accel& arg) const
{
  Accel tmp;
  tmp.linear = M.inverse(arg.linear - p * arg.angular);
  tmp.angular = M.inverse(arg.angular);
  return tmp;
}

inline Wrench Frame::inverse(const Wrench& arg) const
{
  Wrench tmp;
  tmp.force = M.inverse(arg.force);
  tmp.torque = M.inverse(arg.torque - p * arg.force);
  return tmp;
}

inline Vector Frame::operator*(const Vector& arg) const
{
  return M * arg + p;
}

inline Wrench Frame::operator*(const Wrench& arg) const
{
  Wrench tmp;
  tmp.force = M * arg.force;
  tmp.torque = M * arg.torque + p * tmp.force;
  return tmp;
}

inline SegmentInertia Frame::operator*(const SegmentInertia& arg) const
{
  SegmentInertia tmp;
  tmp.linear = M * arg.linear;
  tmp.angular = M * arg.angular + p * tmp.linear;
  return tmp;
}

inline Twist Frame::operator*(const Twist& arg) const
{
  Twist tmp;
  tmp.rot = M * arg.rot;
  tmp.vel = M * arg.vel + p * tmp.rot;
  return tmp;
}

inline Accel Frame::operator*(const Accel& arg) const
{
  Accel tmp;
  tmp.angular = M * arg.angular;
  tmp.linear = M * arg.linear + p * tmp.angular;
  return tmp;
}

inline SegmentJacobian Frame::operator*(const SegmentJacobian& arg) const
{
  SegmentJacobian tmp;
  tmp.angular = M * arg.angular;
  tmp.linear = M * arg.linear + p * tmp.angular;
  return tmp;
}

inline Frame Frame::operator*(const Frame& rhs) const
{
  return Frame(M * rhs.M, M * rhs.p + p);
}

inline Frame Frame::operator-(const Frame& rhs) const
{
  return Frame(M - rhs.M, p - rhs.p);
}

inline std::ostream& operator<<(std::ostream& os, const Frame& arg)
{
  os << "Translation: " << arg.p << ", Rotation: " << arg.M;
  return os;
}
}  // namespace kdl
}  // namespace tobas
