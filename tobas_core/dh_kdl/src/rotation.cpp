#include <dh_std_tools/math.hpp>

#include "../include/dh_kdl/rotation.hpp"
#include "../include/dh_kdl/utilities/utility.hpp"

namespace KDL
{
Rotation Rotation::Quaternion(double x, double y, double z, double w)
{
  double x2, y2, z2, w2;
  x2 = x * x;
  y2 = y * y;
  z2 = z * z;
  w2 = w * w;
  return Rotation(
    w2 + x2 - y2 - z2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y, 2 * x * y + 2 * w * z,
    w2 - x2 + y2 - z2, 2 * y * z - 2 * w * x, 2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x,
    w2 - x2 - y2 + z2);
}

void Rotation::getQuaternion(double& x, double& y, double& z, double& w) const
{
  const double trace = data.trace();
  constexpr double epsilon = 1E-12;
  if (trace > epsilon)
  {
    double s = 0.5 / sqrt(trace + 1.0);
    w = 0.25 / s;
    x = ((*this)(2, 1) - (*this)(1, 2)) * s;
    y = ((*this)(0, 2) - (*this)(2, 0)) * s;
    z = ((*this)(1, 0) - (*this)(0, 1)) * s;
  }
  else
  {
    if ((*this)(0, 0) > (*this)(1, 1) && (*this)(0, 0) > (*this)(2, 2))
    {
      double s = 2.0 * sqrt(1.0 + (*this)(0, 0) - (*this)(1, 1) - (*this)(2, 2));
      w = ((*this)(2, 1) - (*this)(1, 2)) / s;
      x = 0.25 * s;
      y = ((*this)(0, 1) + (*this)(1, 0)) / s;
      z = ((*this)(0, 2) + (*this)(2, 0)) / s;
    }
    else if ((*this)(1, 1) > (*this)(2, 2))
    {
      double s = 2.0 * sqrt(1.0 + (*this)(1, 1) - (*this)(0, 0) - (*this)(2, 2));
      w = ((*this)(0, 2) - (*this)(2, 0)) / s;
      x = ((*this)(0, 1) + (*this)(1, 0)) / s;
      y = 0.25 * s;
      z = ((*this)(1, 2) + (*this)(2, 1)) / s;
    }
    else
    {
      double s = 2.0 * sqrt(1.0 + (*this)(2, 2) - (*this)(0, 0) - (*this)(1, 1));
      w = ((*this)(1, 0) - (*this)(0, 1)) / s;
      x = ((*this)(0, 2) + (*this)(2, 0)) / s;
      y = ((*this)(1, 2) + (*this)(2, 1)) / s;
      z = 0.25 * s;
    }
  }
}

Rotation Rotation::RPY(double roll, double pitch, double yaw)
{
  double ca1, cb1, cc1, sa1, sb1, sc1;
  ca1 = cos(yaw);
  sa1 = sin(yaw);
  cb1 = cos(pitch);
  sb1 = sin(pitch);
  cc1 = cos(roll);
  sc1 = sin(roll);
  return Rotation(
    ca1 * cb1, ca1 * sb1 * sc1 - sa1 * cc1, ca1 * sb1 * cc1 + sa1 * sc1, sa1 * cb1,
    sa1 * sb1 * sc1 + ca1 * cc1, sa1 * sb1 * cc1 - ca1 * sc1, -sb1, cb1 * sc1, cb1 * cc1);
}

// Gives back a rotation matrix specified with RPY convention
void Rotation::getRPY(double& roll, double& pitch, double& yaw) const
{
  constexpr double epsilon = 1E-12;
  pitch = atan2(-data(2, 0), sqrt(sqr(data(0, 0)) + sqr(data(1, 0))));
  if (fabs(pitch) > (M_PI_2 - epsilon))
  {
    yaw = atan2(-data(0, 1), data(1, 1));
    roll = 0;
  }
  else
  {
    roll = atan2(data(2, 1), data(2, 2));
    yaw = atan2(data(1, 0), data(0, 1));
  }
}

Rotation Rotation::Rot(const Vector& rotaxis, double angle)
{
  return Rotation::Rot2(rotaxis.normalized(), angle);
}

Rotation Rotation::Rot2(const Vector& rotvec, double angle)
{
  // rotvec must be normalized
  assert(dh_std::isClose(rotvec.norm(), 1));

  // The formula
  // V.(V.tr) + st*[V x] + ct*(I-V.(V.tr))
  // can be found by multiplying it with an arbitrary vector p
  // and noting that this vector is rotated.
  double ct = cos(angle);
  double st = sin(angle);
  double vt = 1 - ct;
  double m_vt_0 = vt * rotvec.x();
  double m_vt_1 = vt * rotvec.y();
  double m_vt_2 = vt * rotvec.z();
  double m_st_0 = rotvec.x() * st;
  double m_st_1 = rotvec.y() * st;
  double m_st_2 = rotvec.z() * st;
  double m_vt_0_1 = m_vt_0 * rotvec.y();
  double m_vt_0_2 = m_vt_0 * rotvec.z();
  double m_vt_1_2 = m_vt_1 * rotvec.z();
  return Rotation(
    ct + m_vt_0 * rotvec.x(), -m_st_2 + m_vt_0_1, m_st_1 + m_vt_0_2, m_st_2 + m_vt_0_1,
    ct + m_vt_1 * rotvec.y(), -m_st_0 + m_vt_1_2, -m_st_1 + m_vt_0_2, m_st_0 + m_vt_1_2,
    ct + m_vt_2 * rotvec.z());
}

Vector Rotation::getRot() const
{
  Vector axis;
  double angle;
  angle = Rotation::getRotAngle(axis, kDefaultEpsilon);
  return axis * angle;
}

double Rotation::getRotAngle(Vector& axis, double eps) const
{
  double angle, x, y, z;       // variables for result
  double epsilon = eps;        // margin to allow for rounding errors
  double epsilon2 = eps * 10;  // margin to distinguish between 0 and 180 degrees

  // optional check that input is pure rotation, 'isRotationMatrix' is defined at:
  // http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/

  if (
    (abs(data(0, 1) - data(1, 0)) < epsilon) && (abs(data(0, 2) - data(2, 0)) < epsilon)
    && (abs(data(1, 2) - data(2, 1)) < epsilon))
  {
    // singularity found
    // first check for identity matrix which must have +1 for all terms
    //  in leading diagonal and zero in other terms
    if (
      (abs(data(0, 1) + data(1, 0)) < epsilon2) && (abs(data(0, 2) + data(2, 0)) < epsilon2)
      && (abs(data(1, 2) + data(2, 1)) < epsilon2)
      && (abs(data(0, 0) + data(1, 1) + data(2, 2) - 3) < epsilon2))
    {
      // this singularity is identity matrix so angle = 0, axis is arbitrary
      // Choose 0, 0, 1 to pass orocos tests
      axis = Vector(0, 0, 1);
      angle = 0;
      return angle;
    }

    // otherwise this singularity is angle = 180
    angle = M_PI;
    double xx = (data(0, 0) + 1) / 2;
    double yy = (data(1, 1) + 1) / 2;
    double zz = (data(2, 2) + 1) / 2;
    double xy = (data(0, 1) + data(1, 0)) / 4;
    double xz = (data(0, 2) + data(2, 0)) / 4;
    double yz = (data(1, 2) + data(2, 1)) / 4;

    if ((xx > yy) && (xx > zz))
    {
      // data(0, 0) is the largest diagonal term
      x = sqrt(xx);
      y = xy / x;
      z = xz / x;
    }
    else if (yy > zz)
    {
      // data(1, 1) is the largest diagonal term
      y = sqrt(yy);
      x = xy / y;
      z = yz / y;
    }
    else
    {
      // data(2, 2) is the largest diagonal term so base result on this
      z = sqrt(zz);
      x = xz / z;
      y = yz / z;
    }
    axis = Vector(x, y, z);
    return angle;  // return 180 deg rotation
  }

  double f = (data.trace() - 1) / 2;

  x = (data(2, 1) - data(1, 2));
  y = (data(0, 2) - data(2, 0));
  z = (data(1, 0) - data(0, 1));
  axis = Vector(x, y, z);
  angle = atan2(axis.norm() / 2, f);
  axis.normalize();
  return angle;
}
}  // namespace KDL
