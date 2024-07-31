#pragma once

#include "./twist.hpp"
#include "./accel.hpp"
#include "./wrench.hpp"
#include "./segmentjacobian.hpp"

namespace kdl
{
class Rotation;
using RotationMap = std::map<std::string, Rotation>;

class Rotation
{
  static constexpr double kEpsilon = 1e-12;

public:
  Eigen::Matrix3d data;

  inline explicit Rotation();
  inline explicit Rotation(
    double Xx,
    double Yx,
    double Zx,
    double Xy,
    double Yy,
    double Zy,
    double Xz,
    double Yz,
    double Zz);
  inline explicit Rotation(const Vector& x, const Vector& y, const Vector& z);
  inline explicit Rotation(const Eigen::Matrix3d& _data);

  // Gives back an identity rotaton matrix
  inline static Rotation Identity();

  inline void setIdentity();

  // Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set
  inline double& operator()(int i, int j);
  // Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set
  inline double operator()(int i, int j) const;

  // Sets the value of *this to its inverse.
  inline void setInverse();
  // Gives back the inverse rotation matrix of *this.
  inline Rotation inverse() const;
  // The same as R.inverse()*v but more efficient.
  inline Vector inverse(const Vector& v) const;
  // The same as R.inverse()*arg but more efficient.
  inline Twist inverse(const Twist& arg) const;
  // The same as R.inverse()*arg but more efficient.
  inline Accel inverse(const Accel& arg) const;
  // The same as R.inverse()*arg but more efficient.
  inline Wrench inverse(const Wrench& arg) const;
  // The same as R.inverse()*arg but more efficient.
  inline SegmentJacobian inverse(const SegmentJacobian& arg) const;

  // The DoRot... functions apply a rotation R to *this,such that *this = *this * Rot..
  // DoRot... functions are only defined when they can be executed more efficiently
  void doRotX(double angle);
  // The DoRot... functions apply a rotation R to *this,such that *this = *this * Rot..
  // DoRot... functions are only defined when they can be executed more efficiently
  void doRotY(double angle);
  // The DoRot... functions apply a rotation R to *this,such that *this = *this * Rot..
  // DoRot... functions are only defined when they can be executed more efficiently
  void doRotZ(double angle);

  // The Rot... static functions give the value of the appropriate rotation matrix back.
  static Rotation RotX(double angle);
  // The Rot... static functions give the value of the appropriate rotation matrix back.
  static Rotation RotY(double angle);
  // The Rot... static functions give the value of the appropriate rotation matrix back.
  static Rotation RotZ(double angle);

  // Along an arbitrary axes.  It is not necessary to normalize rotvec.
  // returns identity rotation matrix in the case that the norm of rotvec
  // is to small to be used.
  // @see Rot2 if you want to handle this error in another way.
  static Rotation Rot(const Vector& rotvec, double angle);

  // Along an arbitrary axes. rotvec must be normalized.
  static Rotation Rot2(const Vector& rotvec, double angle);

  // Returns a vector with the direction of the equiv. axis
  // and its norm is angle
  Vector getRot() const;

  /** Returns the rotation angle around the equiv. axis
   * @param axis the rotation axis is returned in this variable
   * @param eps :  in the case of angle == 0 : rot axis is undefined and chosen
   *                                         to be +/- Z-axis
   *               in the case of angle == PI : 2 solutions, positive Z-component
   *                                            of the axis is chosen.
   * @result returns the rotation angle (between [0..PI] )
   */
  double getRotAngle(Vector& axis) const;

  // Gives back a rotation matrix specified with Quaternion convention
  // the norm of (x,y,z,w) should be equal to 1
  static Rotation Quaternion(double x, double y, double z, double w);

  // Get the quaternion of this matrix
  // \post the norm of (x,y,z,w) is 1
  void getQuaternion(double& x, double& y, double& z, double& w) const;

  /**
   *
   * Gives back a rotation matrix specified with RPY convention:
   * first rotate around X with roll, then around the
   *              old Y with pitch, then around old Z with yaw
   *
   * Invariants:
   *  - RPY(roll,pitch,yaw) == RPY( roll +/- PI, PI-pitch, yaw +/- PI )
   *  - angles + 2*k*PI
   */
  static Rotation RPY(double roll, double pitch, double yaw);

  /**  Gives back a vector in RPY coordinates, variables are bound by
       -  -PI <= roll <= PI
       -   -PI <= Yaw  <= PI
       -  -PI/2 <= PITCH <= PI/2

     convention :
     - first rotate around X with roll,
     - then around the old Y with pitch,
     - then around old Z with yaw

     if pitch == PI/2 or pitch == -PI/2, multiple solutions for gamma and alpha exist.  The solution
  where roll==0 is chosen.

     Invariants:
     - RPY(roll,pitch,yaw) == RPY( roll +/- PI, PI-pitch, yaw +/- PI )
     - angles + 2*k*PI

  **/
  void getRPY(double& roll, double& pitch, double& yaw) const;

  inline double getYaw() const;

  inline double trace() const;

  inline Rotation operator*(const Rotation& rhs) const;
  inline Vector operator*(const Vector& rhs) const;
  inline Twist operator*(const Twist& rhs) const;
  inline Accel operator*(const Accel& rhs) const;
  inline Wrench operator*(const Wrench& rhs) const;
  inline SegmentJacobian operator*(const SegmentJacobian& rhs) const;

  /* Compute the difference of two rotations wrt. the same frame. */
  inline Rotation operator-(const Rotation& rhs) const;

  // Access to the underlying unitvectors of the rotation matrix
  inline Vector UnitX() const;
  // Access to the underlying unitvectors of the rotation matrix
  inline void UnitX(const Vector& X);
  // Access to the underlying unitvectors of the rotation matrix
  inline Vector UnitY() const;
  // Access to the underlying unitvectors of the rotation matrix
  inline void UnitY(const Vector& X);
  // Access to the underlying unitvectors of the rotation matrix
  inline Vector UnitZ() const;
  // Access to the underlying unitvectors of the rotation matrix
  inline void UnitZ(const Vector& X);

  friend std::ostream& operator<<(std::ostream& os, const Rotation& arg);
};

inline Rotation::Rotation()
{
  *this = Rotation::Identity();
}

inline Rotation::Rotation(
  double Xx,
  double Yx,
  double Zx,
  double Xy,
  double Yy,
  double Zy,
  double Xz,
  double Yz,
  double Zz)
{
  data(0, 0) = Xx;
  data(0, 1) = Yx;
  data(0, 2) = Zx;
  data(1, 0) = Xy;
  data(1, 1) = Yy;
  data(1, 2) = Zy;
  data(2, 0) = Xz;
  data(2, 1) = Yz;
  data(2, 2) = Zz;
}

inline Rotation::Rotation(const Vector& x, const Vector& y, const Vector& z)
{
  data.col(0) = x.data;
  data.col(1) = y.data;
  data.col(2) = z.data;
}

inline Rotation::Rotation(const Eigen::Matrix3d& _data) : data(_data)
{
}

inline Rotation Rotation::Identity()
{
  return Rotation(Eigen::Matrix3d::Identity());
}

inline void Rotation::setIdentity()
{
  data.setIdentity();
}

inline Rotation Rotation::operator*(const Rotation& rhs) const
{
  return Rotation(data * rhs.data);  // TODO: SO3を維持するための処理
}

inline Vector Rotation::operator*(const Vector& rhs) const
{
  return Vector(data * rhs.data);
}

inline Twist Rotation::operator*(const Twist& rhs) const
{
  return Twist((*this) * rhs.vel, (*this) * rhs.rot);
}

inline Accel Rotation::operator*(const Accel& rhs) const
{
  return Accel((*this) * rhs.linear, (*this) * rhs.angular);
}

inline Wrench Rotation::operator*(const Wrench& rhs) const
{
  return Wrench((*this) * rhs.force, (*this) * rhs.torque);
}

inline SegmentJacobian Rotation::operator*(const SegmentJacobian& rhs) const
{
  return SegmentJacobian((*this) * rhs.linear, (*this) * rhs.angular);
}

inline Rotation Rotation::operator-(const Rotation& rhs) const
{
  return (*this) * rhs.inverse();
}

inline Rotation Rotation::inverse() const
{
  return Rotation(data.transpose());
}

inline Vector Rotation::inverse(const Vector& v) const
{
  return Vector(data.transpose() * v.data);
}

inline Twist Rotation::inverse(const Twist& arg) const
{
  return Twist(inverse(arg.vel), inverse(arg.rot));
}

inline Accel Rotation::inverse(const Accel& arg) const
{
  return Accel(inverse(arg.linear), inverse(arg.angular));
}

inline Wrench Rotation::inverse(const Wrench& arg) const
{
  return Wrench(inverse(arg.force), inverse(arg.torque));
}

inline SegmentJacobian Rotation::inverse(const SegmentJacobian& arg) const
{
  return SegmentJacobian(inverse(arg.linear), inverse(arg.angular));
}

inline double& Rotation::operator()(int i, int j)
{
  assert(0 <= i && i <= 2 && 0 <= j && j <= 2);
  return data(i, j);
}

inline double Rotation::operator()(int i, int j) const
{
  assert(0 <= i && i <= 2 && 0 <= j && j <= 2);
  return data(i, j);
}

inline void Rotation::setInverse()
{
  data.transposeInPlace();
}

inline double Rotation::getYaw() const
{
  return ::atan2(data(1, 0), data(0, 0));
}

inline double Rotation::trace() const
{
  return data.trace();
}

inline Vector Rotation::UnitX() const
{
  return Vector(data.col(0));
}

inline void Rotation::UnitX(const Vector& X)
{
  data.col(0) = X.data;
}

inline Vector Rotation::UnitY() const
{
  return Vector(data.col(1));
}

inline void Rotation::UnitY(const Vector& X)
{
  data.col(1) = X.data;
}

inline Vector Rotation::UnitZ() const
{
  return Vector(data.col(2));
}

inline void Rotation::UnitZ(const Vector& X)
{
  data.col(2) = X.data;
}
}  // namespace kdl
