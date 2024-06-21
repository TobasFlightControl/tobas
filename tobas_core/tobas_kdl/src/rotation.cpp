#include <tobas_math/linalg.hpp>
#include <tobas_std_tools/float.hpp>

#include "../include/tobas_kdl/rotation.hpp"
#include "../include/tobas_kdl/utilities/utility.hpp"

namespace kdl
{
Rotation Rotation::Quaternion(double x, double y, double z, double w)
{
  assert(tobas_std::isClose(math::norm(x, y, z, w), 1));

  const auto tx = 2 * x;
  const auto ty = 2 * y;
  const auto tz = 2 * z;
  const auto twx = tx * w;
  const auto twy = ty * w;
  const auto twz = tz * w;
  const auto txx = tx * x;
  const auto txy = ty * x;
  const auto txz = tz * x;
  const auto tyy = ty * y;
  const auto tyz = tz * y;
  const auto tzz = tz * z;

  return Rotation(
    1 - (tyy + tzz), txy - twz, txz + twy, txy + twz, 1 - (txx + tzz), tyz - twx, txz - twy, tyz + twx,
    1 - (txx + tyy));
}

void Rotation::getQuaternion(double& x, double& y, double& z, double& w) const
{
  const auto trace = data.trace();
  if (trace > kEpsilon)
  {
    const auto s = 0.5 / sqrt(trace + 1.0);
    w = 0.25 / s;
    x = ((*this)(2, 1) - (*this)(1, 2)) * s;
    y = ((*this)(0, 2) - (*this)(2, 0)) * s;
    z = ((*this)(1, 0) - (*this)(0, 1)) * s;
  }
  else
  {
    if ((*this)(0, 0) > (*this)(1, 1) && (*this)(0, 0) > (*this)(2, 2))
    {
      const auto s = 2.0 * sqrt(1.0 + (*this)(0, 0) - (*this)(1, 1) - (*this)(2, 2));
      w = ((*this)(2, 1) - (*this)(1, 2)) / s;
      x = 0.25 * s;
      y = ((*this)(0, 1) + (*this)(1, 0)) / s;
      z = ((*this)(0, 2) + (*this)(2, 0)) / s;
    }
    else if ((*this)(1, 1) > (*this)(2, 2))
    {
      const auto s = 2.0 * sqrt(1.0 + (*this)(1, 1) - (*this)(0, 0) - (*this)(2, 2));
      w = ((*this)(0, 2) - (*this)(2, 0)) / s;
      x = ((*this)(0, 1) + (*this)(1, 0)) / s;
      y = 0.25 * s;
      z = ((*this)(1, 2) + (*this)(2, 1)) / s;
    }
    else
    {
      const auto s = 2.0 * sqrt(1.0 + (*this)(2, 2) - (*this)(0, 0) - (*this)(1, 1));
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
    ca1 * cb1, ca1 * sb1 * sc1 - sa1 * cc1, ca1 * sb1 * cc1 + sa1 * sc1, sa1 * cb1, sa1 * sb1 * sc1 + ca1 * cc1,
    sa1 * sb1 * cc1 - ca1 * sc1, -sb1, cb1 * sc1, cb1 * cc1);
}

void Rotation::getRPY(double& roll, double& pitch, double& yaw) const
{
  pitch = atan2(-data(2, 0), math::norm(data(0, 0), data(1, 0)));
  if (fabs(pitch) > (M_PI_2 - kEpsilon))
  {
    yaw = atan2(-data(0, 1), data(1, 1));
    roll = 0;
  }
  else
  {
    roll = atan2(data(2, 1), data(2, 2));
    yaw = atan2(data(1, 0), data(0, 0));
  }
}

void Rotation::doRotX(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  const auto x1 = cs * (*this)(0, 1) + sn * (*this)(0, 2);
  const auto x2 = cs * (*this)(1, 1) + sn * (*this)(1, 2);
  const auto x3 = cs * (*this)(2, 1) + sn * (*this)(2, 2);
  (*this)(0, 2) = -sn * (*this)(0, 1) + cs * (*this)(0, 2);
  (*this)(1, 2) = -sn * (*this)(1, 1) + cs * (*this)(1, 2);
  (*this)(2, 2) = -sn * (*this)(2, 1) + cs * (*this)(2, 2);
  (*this)(0, 1) = x1;
  (*this)(1, 1) = x2;
  (*this)(2, 1) = x3;
}

void Rotation::doRotY(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  const auto x1 = cs * (*this)(0, 0) - sn * (*this)(0, 2);
  const auto x2 = cs * (*this)(1, 0) - sn * (*this)(1, 2);
  const auto x3 = cs * (*this)(2, 0) - sn * (*this)(2, 2);
  (*this)(0, 2) = sn * (*this)(0, 0) + cs * (*this)(0, 2);
  (*this)(1, 2) = sn * (*this)(1, 0) + cs * (*this)(1, 2);
  (*this)(2, 2) = sn * (*this)(2, 0) + cs * (*this)(2, 2);
  (*this)(0, 0) = x1;
  (*this)(1, 0) = x2;
  (*this)(2, 0) = x3;
}

void Rotation::doRotZ(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  const auto x1 = cs * (*this)(0, 0) + sn * (*this)(0, 1);
  const auto x2 = cs * (*this)(1, 0) + sn * (*this)(1, 1);
  const auto x3 = cs * (*this)(2, 0) + sn * (*this)(2, 1);
  (*this)(0, 1) = -sn * (*this)(0, 0) + cs * (*this)(0, 1);
  (*this)(1, 1) = -sn * (*this)(1, 0) + cs * (*this)(1, 1);
  (*this)(2, 1) = -sn * (*this)(2, 0) + cs * (*this)(2, 1);
  (*this)(0, 0) = x1;
  (*this)(1, 0) = x2;
  (*this)(2, 0) = x3;
}

Rotation Rotation::RotX(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  return Rotation(1, 0, 0, 0, cs, -sn, 0, sn, cs);
}

Rotation Rotation::RotY(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  return Rotation(cs, 0, sn, 0, 1, 0, -sn, 0, cs);
}

Rotation Rotation::RotZ(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  return Rotation(cs, -sn, 0, sn, cs, 0, 0, 0, 1);
}

Rotation Rotation::Rot(const Vector& rotaxis, double angle)
{
  return Rotation::Rot2(rotaxis.normalized(), angle);
}

Rotation Rotation::Rot2(const Vector& rotvec, double angle)
{
  // rotvec must be normalized
  assert(tobas_std::isClose(rotvec.norm(), 1));

  // The formula
  // V.(V.tr) + st*[V x] + ct*(I-V.(V.tr))
  // can be found by multiplying it with an arbitrary vector p
  // and noting that this vector is rotated.
  const auto ct = cos(angle);
  const auto st = sin(angle);
  const auto vt = 1 - ct;
  const auto m_vt_0 = vt * rotvec.x();
  const auto m_vt_1 = vt * rotvec.y();
  const auto m_vt_2 = vt * rotvec.z();
  const auto m_st_0 = rotvec.x() * st;
  const auto m_st_1 = rotvec.y() * st;
  const auto m_st_2 = rotvec.z() * st;
  const auto m_vt_0_1 = m_vt_0 * rotvec.y();
  const auto m_vt_0_2 = m_vt_0 * rotvec.z();
  const auto m_vt_1_2 = m_vt_1 * rotvec.z();
  return Rotation(
    ct + m_vt_0 * rotvec.x(), -m_st_2 + m_vt_0_1, m_st_1 + m_vt_0_2, m_st_2 + m_vt_0_1, ct + m_vt_1 * rotvec.y(),
    -m_st_0 + m_vt_1_2, -m_st_1 + m_vt_0_2, m_st_0 + m_vt_1_2, ct + m_vt_2 * rotvec.z());
}

Vector Rotation::getRot() const
{
  Vector axis;
  const auto angle = Rotation::getRotAngle(axis);
  return axis * angle;
}

double Rotation::getRotAngle(Vector& axis) const
{
  double angle, x, y, z;                    // variables for result
  constexpr auto epsilon2 = kEpsilon * 10;  // margin to distinguish between 0 and 180 degrees

  // optional check that input is pure rotation, 'isRotationMatrix' is defined at:
  // http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/

  if (
    (abs(data(0, 1) - data(1, 0)) < kEpsilon) && (abs(data(0, 2) - data(2, 0)) < kEpsilon)
    && (abs(data(1, 2) - data(2, 1)) < kEpsilon))
  {
    // singularity found
    // first check for identity matrix which must have +1
    // for all terms in leading diagonal and zero in other terms
    if (
      (abs(data(0, 1) + data(1, 0)) < epsilon2) && (abs(data(0, 2) + data(2, 0)) < epsilon2)
      && (abs(data(1, 2) + data(2, 1)) < epsilon2) && (abs(data(0, 0) + data(1, 1) + data(2, 2) - 3) < epsilon2))
    {
      // this singularity is identity matrix so angle = 0, axis is arbitrary
      // Choose 0, 0, 1 to pass orocos tests
      axis = Vector(0, 0, 1);
      angle = 0;
      return angle;
    }

    // otherwise this singularity is angle = 180
    angle = M_PI;
    const auto xx = (data(0, 0) + 1) / 2;
    const auto yy = (data(1, 1) + 1) / 2;
    const auto zz = (data(2, 2) + 1) / 2;
    const auto xy = (data(0, 1) + data(1, 0)) / 4;
    const auto xz = (data(0, 2) + data(2, 0)) / 4;
    const auto yz = (data(1, 2) + data(2, 1)) / 4;

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

  const auto f = (data.trace() - 1) / 2;

  x = (data(2, 1) - data(1, 2));
  y = (data(0, 2) - data(2, 0));
  z = (data(1, 0) - data(0, 1));
  axis = Vector(x, y, z);
  angle = atan2(axis.norm() / 2, f);
  axis.normalize();
  return angle;
}
}  // namespace kdl
