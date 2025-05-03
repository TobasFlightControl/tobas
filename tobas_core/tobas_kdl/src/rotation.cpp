#include <tobas_math/linalg.hpp>
#include <tobas_std_tools/float.hpp>

#include "../include/tobas_kdl/rotation.hpp"
#include "../include/tobas_kdl/utilities/utility.hpp"

#define EPS 1e-12

using namespace std;

namespace kdl
{
Rotation Rotation::Quaternion(double x, double y, double z, double w)
{
  assert(tobas_std::isClose(math::norm(x, y, z, w), 1.));

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
    1 - (tyy + tzz), txy - twz, txz + twy, txy + twz, 1 - (txx + tzz), tyz - twx, txz - twy, tyz + twx, 1 - (txx + tyy));
}

void Rotation::getQuaternion(double& x, double& y, double& z, double& w) const
{
  const auto trace = data.trace();
  if (trace > EPS) {
    const auto s = 0.5 / sqrt(trace + 1.);
    w = 0.25 / s;
    x = (data(2, 1) - data(1, 2)) * s;
    y = (data(0, 2) - data(2, 0)) * s;
    z = (data(1, 0) - data(0, 1)) * s;
  }
  else {
    if (data(0, 0) > data(1, 1) && data(0, 0) > data(2, 2)) {
      const auto s = 2. * sqrt(1. + data(0, 0) - data(1, 1) - data(2, 2));
      w = (data(2, 1) - data(1, 2)) / s;
      x = 0.25 * s;
      y = (data(0, 1) + data(1, 0)) / s;
      z = (data(0, 2) + data(2, 0)) / s;
    }
    else if (data(1, 1) > data(2, 2)) {
      const auto s = 2. * sqrt(1. + data(1, 1) - data(0, 0) - data(2, 2));
      w = (data(0, 2) - data(2, 0)) / s;
      x = (data(0, 1) + data(1, 0)) / s;
      y = 0.25 * s;
      z = (data(1, 2) + data(2, 1)) / s;
    }
    else {
      const auto s = 2. * sqrt(1. + data(2, 2) - data(0, 0) - data(1, 1));
      w = (data(1, 0) - data(0, 1)) / s;
      x = (data(0, 2) + data(2, 0)) / s;
      y = (data(1, 2) + data(2, 1)) / s;
      z = 0.25 * s;
    }
  }
}

Rotation Rotation::RPY(double roll, double pitch, double yaw)
{
  const auto ca = cos(yaw);
  const auto sa = sin(yaw);
  const auto cb = cos(pitch);
  const auto sb = sin(pitch);
  const auto cc = cos(roll);
  const auto sc = sin(roll);

  const auto xx = ca * cb;
  const auto yx = ca * sb * sc - sa * cc;
  const auto zx = ca * sb * cc + sa * sc;
  const auto xy = sa * cb;
  const auto yy = sa * sb * sc + ca * cc;
  const auto zy = sa * sb * cc - ca * sc;
  const auto xz = -sb;
  const auto yz = cb * sc;
  const auto zz = cb * cc;

  return Rotation(xx, yx, zx, xy, yy, zy, xz, yz, zz);
}

void Rotation::getRPY(double& roll, double& pitch, double& yaw) const
{
  pitch = atan2(-data(2, 0), math::norm(data(0, 0), data(1, 0)));
  if (fabs(pitch) > (M_PI_2 - EPS)) {
    yaw = atan2(-data(0, 1), data(1, 1));
    roll = 0.;
  }
  else {
    roll = atan2(data(2, 1), data(2, 2));
    yaw = atan2(data(1, 0), data(0, 0));
  }
}

tuple<double, double, double> Rotation::getRPY() const
{
  double roll, pitch, yaw;
  getRPY(roll, pitch, yaw);
  return { roll, pitch, yaw };
}

void Rotation::doRotX(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  const auto x1 = cs * data(0, 1) + sn * data(0, 2);
  const auto x2 = cs * data(1, 1) + sn * data(1, 2);
  const auto x3 = cs * data(2, 1) + sn * data(2, 2);
  data(0, 2) = -sn * data(0, 1) + cs * data(0, 2);
  data(1, 2) = -sn * data(1, 1) + cs * data(1, 2);
  data(2, 2) = -sn * data(2, 1) + cs * data(2, 2);
  data(0, 1) = x1;
  data(1, 1) = x2;
  data(2, 1) = x3;
}

void Rotation::doRotY(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  const auto x1 = cs * data(0, 0) - sn * data(0, 2);
  const auto x2 = cs * data(1, 0) - sn * data(1, 2);
  const auto x3 = cs * data(2, 0) - sn * data(2, 2);
  data(0, 2) = sn * data(0, 0) + cs * data(0, 2);
  data(1, 2) = sn * data(1, 0) + cs * data(1, 2);
  data(2, 2) = sn * data(2, 0) + cs * data(2, 2);
  data(0, 0) = x1;
  data(1, 0) = x2;
  data(2, 0) = x3;
}

void Rotation::doRotZ(double angle)
{
  const auto cs = cos(angle);
  const auto sn = sin(angle);
  const auto x1 = cs * data(0, 0) + sn * data(0, 1);
  const auto x2 = cs * data(1, 0) + sn * data(1, 1);
  const auto x3 = cs * data(2, 0) + sn * data(2, 1);
  data(0, 1) = -sn * data(0, 0) + cs * data(0, 1);
  data(1, 1) = -sn * data(1, 0) + cs * data(1, 1);
  data(2, 1) = -sn * data(2, 0) + cs * data(2, 1);
  data(0, 0) = x1;
  data(1, 0) = x2;
  data(2, 0) = x3;
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

Rotation Rotation::Rot(const Vector& axis, double angle)
{
  // Axis must be normalized
  assert(tobas_std::isClose(axis.norm(), 1.));

  // The formula
  // R(n, θ) = n n^T + (E - n n^T)) cos(θ) + skew(n) sin(θ) = E + skew(n) sin(θ) + skew(n)^2 (1 - cos(θ))
  // can be found by multiplying it with an arbitrary vector p and noting that this vector is rotated.
  const auto ct = cos(angle);
  const auto st = sin(angle);
  const auto vt = 1 - ct;
  const auto m_vt_0 = vt * axis.x();
  const auto m_vt_1 = vt * axis.y();
  const auto m_vt_2 = vt * axis.z();
  const auto m_st_0 = axis.x() * st;
  const auto m_st_1 = axis.y() * st;
  const auto m_st_2 = axis.z() * st;
  const auto m_vt_0_1 = m_vt_0 * axis.y();
  const auto m_vt_0_2 = m_vt_0 * axis.z();
  const auto m_vt_1_2 = m_vt_1 * axis.z();

  return Rotation(
    ct + m_vt_0 * axis.x(), -m_st_2 + m_vt_0_1, m_st_1 + m_vt_0_2, m_st_2 + m_vt_0_1, ct + m_vt_1 * axis.y(),
    -m_st_0 + m_vt_1_2, -m_st_1 + m_vt_0_2, m_st_0 + m_vt_1_2, ct + m_vt_2 * axis.z());
}

Rotation Rotation::Rot(const Vector& vec)
{
  return Rotation::Rot(vec.normalized(), vec.norm());
}

Vector Rotation::getRot() const
{
  const auto [angle, axis] = Rotation::getAngleAxis();
  return angle * axis;
}

pair<double, Vector> Rotation::getAngleAxis() const
{
  constexpr auto eps2 = EPS * 10;  // margin to distinguish between 0 and 180 degrees

  // Optional check that input is pure rotation, 'isRotationMatrix' is defined at:
  // http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/

  if (fabs(data(0, 1) - data(1, 0) < EPS) && fabs(data(0, 2) - data(2, 0)) < EPS && fabs(data(1, 2) - data(2, 1)) < EPS) {
    // Singularity found
    // First check for identity matrix which must have +1
    // for all terms in leading diagonal and zero in other terms
    if (
      fabs(data(0, 1) + data(1, 0)) < eps2 && fabs(data(0, 2) + data(2, 0)) < eps2 &&
      fabs(data(1, 2) + data(2, 1)) < eps2 && fabs(this->trace() - 3) < eps2) {
      // This singularity is identity matrix so angle = 0, axis is arbitrary chose.
      return { 0., Vector::UnitZ() };
    }

    // otherwise this singularity is angle = 180
    const auto xx = (data(0, 0) + 1) / 2;
    const auto yy = (data(1, 1) + 1) / 2;
    const auto zz = (data(2, 2) + 1) / 2;
    const auto xy = (data(0, 1) + data(1, 0)) / 4;
    const auto xz = (data(0, 2) + data(2, 0)) / 4;
    const auto yz = (data(1, 2) + data(2, 1)) / 4;

    double x, y, z;
    if (xx > yy && xx > zz) {
      // data(0, 0) is the largest diagonal term
      x = sqrt(xx);
      y = xy / x;
      z = xz / x;
    }
    else if (yy > zz) {
      // data(1, 1) is the largest diagonal term
      y = sqrt(yy);
      x = xy / y;
      z = yz / y;
    }
    else {
      // data(2, 2) is the largest diagonal term so base result on this
      z = sqrt(zz);
      x = xz / z;
      y = yz / z;
    }

    return { M_PI, Vector(x, y, z) };  // return 180 deg rotation
  }

  const auto f = (data.trace() - 1) / 2;

  const auto x = data(2, 1) - data(1, 2);
  const auto y = data(0, 2) - data(2, 0);
  const auto z = data(1, 0) - data(0, 1);
  const Vector axis(x, y, z);
  const auto angle = atan2(axis.norm() / 2, f);
  return { angle, axis.normalized() };
}
}  // namespace kdl
