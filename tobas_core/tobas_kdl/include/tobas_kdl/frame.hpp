#pragma once

#include "./vector.hpp"
#include "./rotation.hpp"
#include "./twist.hpp"
#include "./accel.hpp"
#include "./wrench.hpp"
#include "./impulse.hpp"

namespace kdl
{
class Frame;
using FrameMap = std::map<std::string, Frame>;

/**
  \brief represents a frame transformation in 3D space (rotation + translation)

    if V2 = Frame*V1 (V2 expressed in frame A, V1 expressed in frame B)
    then V2 = Frame.M*V1+Frame.p

    Frame.M contains columns that represent the axes of frame B wrt frame A
    Frame.p contains the origin of frame B expressed in frame A.
*/
class Frame
{
public:
  Vector p;    // Origine of the Frame
  Rotation M;  // Orientation of the Frame

  inline explicit Frame(const Rotation& R, const Vector& V);
  // The rotation matrix defaults to identity
  inline explicit Frame(const Vector& V);
  // The position matrix defaults to zero
  inline explicit Frame(const Rotation& R);
  inline explicit Frame();

  // @return the identity transformation Frame(Rotation::Identity(),Vector::Zero()).
  inline static Frame Identity();

  /*
  // DH_Craig1989 : constructs a transformationmatrix
  // T_link(i-1)_link(i) with the Denavit-Hartenberg convention as
  // described in the Craigs book: Craig, J. J.,Introduction to
  // Robotics: Mechanics and Control, Addison-Wesley,
  // isbn:0-201-10326-5, 1986.
  //
  // Note that the frame is a redundant way to express the information
  // in the DH-convention.
  // \verbatim
  // Parameters in full : a(i-1),alpha(i-1),d(i),theta(i)
  //
  //  axis i-1 is connected by link i-1 to axis i numbering axis 1
  //  to axis n link 0 (immobile base) to link n
  //
  //  link length a(i-1) length of the mutual perpendicular line
  //  (normal) between the 2 axes.  This normal runs from (i-1) to
  //  (i) axis.
  //
  //  link twist alpha(i-1): construct plane perpendicular to the
  //  normal project axis(i-1) and axis(i) into plane angle from
  //  (i-1) to (i) measured in the direction of the normal
  //
  //  link offset d(i) signed distance between normal (i-1) to (i)
  //  and normal (i) to (i+1) along axis i joint angle theta(i)
  //  signed angle between normal (i-1) to (i) and normal (i) to
  //  (i+1) along axis i
  //
  //   First and last joints : a(0)= a(n) = 0
  //   alpha(0) = alpha(n) = 0
  //
  //   PRISMATIC : theta(1) = 0 d(1) arbitrarily
  //
  //   REVOLUTE : theta(1) arbitrarily d(1) = 0
  //
  //   Not unique : if intersecting joint axis 2 choices for normal
  //   Frame assignment of the DH convention : Z(i-1) follows axis
  //   (i-1) X(i-1) is the normal between axis(i-1) and axis(i)
  //   Y(i-1) follows out of Z(i-1) and X(i-1)
  //
  //     a(i-1)     = distance from Z(i-1) to Z(i) along X(i-1)
  //     alpha(i-1) = angle between Z(i-1) to Z(i) along X(i-1)
  //     d(i)       = distance from X(i-1) to X(i) along Z(i)
  //     theta(i)   = angle between X(i-1) to X(i) along X(i)
  // \endverbatim
  */
  static Frame DH_Craig1989(double a, double alpha, double d, double theta);

  // DH : constructs a transformationmatrix T_link(i-1)_link(i) with
  // the Denavit-Hartenberg convention as described in the original
  // publictation: Denavit, J. and Hartenberg, R. S., A kinematic
  // notation for lower-pair mechanisms based on matrices, ASME
  // Journal of Applied Mechanics, 23:215-221, 1955.
  static Frame DH(double a, double alpha, double d, double theta);

  // Reads data from an double array
  //\TODO should be formulated as a constructor
  void Make4x4(double* d);

  inline void setIdentity();

  // Treats a frame as a 4x4 matrix and returns element i,j
  // Access to elements 0..3,0..3, bounds are checked when NDEBUG is not set
  inline double operator()(int i, int j);
  // Treats a frame as a 4x4 matrix and returns element i,j
  // Access to elements 0..3,0..3, bounds are checked when NDEBUG is not set
  inline double operator()(int i, int j) const;

  // = inverse
  // Gives back inverse transformation of a Frame
  inline Frame inverse() const;
  // The same as p2=R.inverse()*p but more efficient.
  inline Vector inverse(const Vector& arg) const;
  // The same as p2=R.inverse()*p but more efficient.
  inline Wrench inverse(const Wrench& arg) const;
  // The same as p2=R.inverse()*p but more efficient.
  inline Twist inverse(const Twist& arg) const;
  // The same as p2=R.inverse()*p but more efficient.
  inline Accel inverse(const Accel& arg) const;

  inline Vector operator*(const Vector& arg) const;
  inline Wrench operator*(const Wrench& arg) const;
  inline SegmentInertia operator*(const SegmentInertia& arg) const;
  inline Twist operator*(const Twist& arg) const;
  inline Accel operator*(const Accel& arg) const;
  inline SegmentJacobian operator*(const SegmentJacobian& arg) const;
  inline Frame operator*(const Frame& rhs) const;

  /* Compute the difference of two frames wrt. the same frame. */
  inline Frame operator-(const Frame& rhs) const;

  // The twist <t_this> is expressed wrt the current
  // frame.  This frame is integrated into an updated frame with
  // <samplefrequency>.  Very simple first order integration rule.
  void integrate(const Twist& t_this, double frequency);

  /* フレームを6次元ベクトルに変換． */
  Twist toTwist() const;

  friend std::ostream& operator<<(std::ostream& os, const Frame& arg);
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

inline void Frame::setIdentity()
{
  p.setZero();
  M.setIdentity();
}

inline double Frame::operator()(int i, int j)
{
  assert((0 <= i) && (i <= 3) && (0 <= j) && (j <= 3));
  if (i == 3)
  {
    if (j == 3)
      return 1.0;
    else
      return 0.0;
  }
  else
  {
    if (j == 3)
      return p(i);
    else
      return M(i, j);
  }
}

inline double Frame::operator()(int i, int j) const
{
  assert((0 <= i) && (i <= 3) && (0 <= j) && (j <= 3));
  if (i == 3)
  {
    if (j == 3)
      return 1;
    else
      return 0;
  }
  else
  {
    if (j == 3)
      return p(i);
    else
      return M(i, j);
  }
}

inline Wrench Frame::inverse(const Wrench& arg) const
{
  Wrench tmp;
  tmp.force = M.inverse(arg.force);
  tmp.torque = M.inverse(arg.torque - p * arg.force);
  return tmp;
}

inline Twist Frame::inverse(const Twist& arg) const
{
  Twist tmp;
  tmp.rot = M.inverse(arg.rot);
  tmp.vel = M.inverse(arg.vel - p * arg.rot);
  return tmp;
}

inline Accel Frame::inverse(const Accel& arg) const
{
  Accel tmp;
  tmp.angular = M.inverse(arg.angular);
  tmp.linear = M.inverse(arg.linear - p * arg.angular);
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

inline Vector Frame::inverse(const Vector& arg) const
{
  return M.inverse(arg - p);
}

inline Frame Frame::inverse() const
{
  return Frame(M.inverse(), -M.inverse(p));
}
}  // namespace kdl
