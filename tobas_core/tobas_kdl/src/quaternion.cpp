#include <tobas_std_tools/float.hpp>
#include <tobas_std_tools/geometry.hpp>

#include "../include/tobas_kdl/quaternion.hpp"
#include "../include/tobas_kdl/utilities/utility.hpp"

using namespace std;

namespace kdl
{
Quaternion::Quaternion(const double& _x, const double& _y, const double& _z, const double& _w)
  : x(_x), y(_y), z(_z), w(_w)
{
}

Quaternion::Quaternion(const Rotation& rot)
{
  rot.getQuaternion(x, y, z, w);
}

Quaternion::Quaternion() : x(0), y(0), z(0), w(1)
{
}

Quaternion Quaternion::Identity()
{
  return Quaternion(0, 0, 0, 1);
}

Quaternion Quaternion::AngleAxis(const Vector& a)
{
  const auto angle = a.norm();

  if (angle < numeric_limits<double>::epsilon())
    return Quaternion::Identity();

  const auto axis = a / angle;
  const auto mag = sin(angle / 2.);
  return Quaternion(mag * axis.x(), mag * axis.y(), mag * axis.z(), cos(angle / 2));
}

Quaternion Quaternion::RPY(const double& roll, const double& pitch, const double& yaw)
{
  return Quaternion(Rotation::RPY(roll, pitch, yaw));
}

void Quaternion::getRPY(double& roll, double& pitch, double& yaw) const
{
  tobas_std::quaternionToEuler(x, y, z, w, roll, pitch, yaw);
}

Quaternion Quaternion::complexConjugate() const
{
  return Quaternion(-x, -y, -z, w);
}

Quaternion Quaternion::inverse() const
{
  return complexConjugate() / squaredNorm();
}

double Quaternion::squaredNorm() const
{
  return sqr(x) + sqr(y) + sqr(z) + sqr(w);
}

double Quaternion::norm() const
{
  return sqrt(squaredNorm());
}

Quaternion Quaternion::normalize() const
{
  return *this / norm();
}

bool Quaternion::isNormalized() const
{
  return tobas_std::isClose(squaredNorm(), 1);
}

Quaternion Quaternion::differential(const Vector& angvel) const
{
  const auto a = angvel / 2;
  return *this * Quaternion(a.x(), a.y(), a.z(), 0);
}

Quaternion Quaternion::operator/(const double& rhs) const
{
  assert(rhs != 0);
  return Quaternion(x / rhs, y / rhs, z / rhs, w / rhs);
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const
{
  const auto nw = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
  const auto nx = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
  const auto ny = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
  const auto nz = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
  return Quaternion(nx, ny, nz, nw);
}

Vector Quaternion::operator*(const Vector& v) const
{
  const auto res = *this * Quaternion(v.x(), v.y(), v.z(), 0) * complexConjugate();
  return Vector(res.x, res.y, res.z);
}

ostream& operator<<(ostream& os, const Quaternion& arg)
{
  os << "w: " << arg.w << ", x: " << arg.x << ", y: " << arg.y << ", z: " << arg.z;
  return os;
}
}  // namespace kdl
