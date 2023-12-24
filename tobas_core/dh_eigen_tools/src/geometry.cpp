#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>
#include <dh_std_tools/assert.hpp>

#include "../include/dh_eigen_tools/geometry.hpp"

#define EPS numeric_limits<double>::epsilon()

using namespace std;
using namespace Eigen;

namespace eigen_tools
{
void vectorNedToNwu(const Vector3d& src, Vector3d& des)
{
  des.x() = src.x();
  des.y() = -src.y();
  des.z() = -src.z();
}

void vectorNwuToNed(const Vector3d& src, Vector3d& des)
{
  vectorNedToNwu(src, des);
}

void vectorNedToNwu(Vector3d& arg)
{
  vectorNedToNwu(arg, arg);
}

void vectorNwuToNed(Vector3d& arg)
{
  vectorNwuToNed(arg, arg);
}

AngleAxisd vectorToAngleAxis(const Vector3d& w)
{
  const double angle = w.norm();
  const Vector3d axis = (angle == 0) ? Vector3d::UnitX() : w.normalized();
  return AngleAxisd(angle, axis);
}

Vector3d angleAxisToVector(const AngleAxisd& angle_axis)
{
  return angle_axis.angle() * angle_axis.axis();
}

Matrix3d angleAxisToRotMat(const Vector3d& w)
{
  return vectorToAngleAxis(w).toRotationMatrix();
}

Quaterniond angleAxisToQuaternion(const Vector3d& w)
{
  return Quaterniond(vectorToAngleAxis(w));
}

Vector3d quaternionToAngleAxis(const Quaterniond& q)
{
  AngleAxisd angle_axis(q);
  return angleAxisToVector(angle_axis);
}

Quaterniond quaternionFromRPY(const double& roll, const double& pitch, const double& yaw)
{
  const AngleAxisd rot_yaw(yaw, Vector3d::UnitZ());
  const AngleAxisd rot_pitch(pitch, Vector3d::UnitY());
  const AngleAxisd rot_roll(roll, Vector3d::UnitX());
  return rot_yaw * rot_pitch * rot_roll;
}

Matrix3d dcmFromRPY(const double& roll, const double& pitch, const double& yaw)
{
  return quaternionFromRPY(roll, pitch, yaw).toRotationMatrix();
}

Quaterniond hamiltonToQuaternion(const Vector4d& ham)
{
  return Quaterniond((Vector4d() << ham.block<3, 1>(1, 0), ham.block<1, 1>(0, 0)).finished());
}

Vector4d quaternionToHamilton(const Quaterniond& q)
{
  return (Vector4d() << q.coeffs().block<1, 1>(3, 0), q.coeffs().block<3, 1>(0, 0)).finished();
}

Matrix3d crossMat(const double& x, const double& y, const double& z)
{
  Matrix3d res;
  res << 0, -z, y, z, 0, -x, -y, x, 0;
  return res;
}

Matrix3d crossMat(const Vector3d& v)
{
  return crossMat(v(0), v(1), v(2));
}

Matrix3d crossMat2(const double& x, const double& y, const double& z)
{
  const double xx = x * x;
  const double yy = y * y;
  const double zz = z * z;
  const double xy = x * y;
  const double yz = y * z;
  const double zx = z * x;

  Matrix3d res;
  res << -yy - zz, xy, zx, xy, -zz - xx, yz, zx, yz, -xx - yy;

  return res;
}

Matrix3d crossMat2(const Vector3d& v)
{
  return crossMat2(v(0), v(1), v(2));
}

void imuToQuaternion(const Vector3d& a, const Vector3d& m, const Vector3d& m0, Quaterniond& q)
{
  dh_std::imuToQuaternion(
    a.x(), a.y(), a.z(), m.x(), m.y(), m.z(), m0.x(), m0.y(), m0.z(), q.x(), q.y(), q.z(), q.w());
}

Matrix3d angvelFromEulerrateGlobal(const double& pitch, const double& yaw)
{
  Matrix3d res;

  const double cos_pitch = cos(pitch);
  const double sin_pitch = sin(pitch);
  const double cos_yaw = cos(yaw);
  const double sin_yaw = sin(yaw);

  res(0, 0) = cos_pitch * cos_yaw;
  res(0, 1) = -sin_yaw;
  res(0, 2) = 0;
  res(1, 0) = cos_pitch * sin_yaw;
  res(1, 1) = cos_yaw;
  res(1, 2) = 0;
  res(2, 0) = -sin_pitch;
  res(2, 1) = 0;
  res(2, 2) = 1;

  return res;
}

Vector3d angvelFromEulerrateGlobal(const Vector3d& rpyd, const double& pitch, const double& yaw)
{
  return angvelFromEulerrateGlobal(pitch, yaw) * rpyd;
}

Matrix3d angvelFromEulerrateLocal(const double& roll, const double& pitch)
{
  Matrix3d res;

  const double cos_roll = cos(roll);
  const double sin_roll = sin(roll);
  const double cos_pitch = cos(pitch);
  const double sin_pitch = sin(pitch);

  res(0, 0) = 1;
  res(0, 1) = 0;
  res(0, 2) = -sin_pitch;
  res(1, 0) = 0;
  res(1, 1) = cos_roll;
  res(1, 2) = sin_roll * cos_pitch;
  res(2, 0) = 0;
  res(2, 1) = -sin_roll;
  res(2, 2) = cos_roll * cos_pitch;

  return res;
}

Vector3d angvelFromEulerrateLocal(const Vector3d& rpyd, const double& roll, const double& pitch)
{
  return angvelFromEulerrateLocal(roll, pitch) * rpyd;
}

Matrix3d eulerrateFromAngvelGlobal(const double& pitch, const double& yaw)
{
  Matrix3d res;

  const double cos_pitch = cos(pitch);
  const double tan_pitch = tan(pitch);
  const double cos_yaw = cos(yaw);
  const double sin_yaw = sin(yaw);
  assert(cos_pitch > EPS);

  res(0, 0) = cos_yaw / cos_pitch;
  res(0, 1) = sin_yaw / cos_pitch;
  res(0, 2) = 0;
  res(1, 0) = -sin_yaw;
  res(1, 1) = cos_yaw;
  res(1, 2) = 0;
  res(2, 0) = cos_yaw * tan_pitch;
  res(2, 1) = sin_yaw * tan_pitch;
  res(2, 2) = 1;

  return res;
}

Vector3d eulerrateFromAngvelGlobal(const Vector3d& angvel, const double& pitch, const double& yaw)
{
  return eulerrateFromAngvelGlobal(pitch, yaw) * angvel;
}

Matrix3d eulerrateFromAngvelLocal(const double& roll, const double& pitch)
{
  Matrix3d res;

  const double cos_roll = cos(roll);
  const double sin_roll = sin(roll);
  const double cos_pitch = cos(pitch);
  const double tan_pitch = tan(pitch);
  assertWithMsg(cos_pitch > EPS, "roll: " << roll << ", pitch: " << pitch);

  res(0, 0) = 1;
  res(0, 1) = sin_roll * tan_pitch;
  res(0, 2) = cos_roll * tan_pitch;
  res(1, 0) = 0;
  res(1, 1) = cos_roll;
  res(1, 2) = -sin_roll;
  res(2, 0) = 0;
  res(2, 1) = sin_roll / cos_pitch;
  res(2, 2) = cos_roll / cos_pitch;

  return res;
}

Vector3d eulerrateFromAngvelLocal(const Vector3d& angvel, const double& roll, const double& pitch)
{
  return eulerrateFromAngvelLocal(roll, pitch) * angvel;
}

Matrix3d matrixFromAngleAxis(const Vector3d& a)
{
  const double angle = a.norm();
  const Vector3d axis = a.normalized();

  const Matrix3d axis_cross = eigen_tools::crossMat(axis);
  const Matrix3d axis_cross2 = eigen_tools::crossMat2(axis);
  const Matrix3d I = Matrix3d::Identity();
  const auto data = I + axis_cross * sin(angle) + axis_cross2 * (1 - cos(angle));

  return Matrix3d(data);
}

Vector3d AngleAxisFromMatrix(const Matrix3d& r)
{
  const Vector3d l(r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1));
  const double l_norm = l.norm();
  if (l_norm > EPS)
    return (atan2(l_norm, r.trace() - 1) / l_norm) * l;
  else
    return Vector3d::Zero();
}

Vector3d euleraccFromAngaccGlobal(
  const Vector3d& angvel,
  const Vector3d& angacc,
  const double& pitch,
  const double& yaw)
{
  const double cos_pitch = cos(pitch);
  const double tan_pitch = tan(pitch);
  const double cos_yaw = cos(yaw);
  const double sin_yaw = sin(yaw);

  const Vector3d rpyd = eulerrateFromAngvelGlobal(angvel, pitch, yaw);

  Vector3d rpydd;
  rpydd.x() = rpyd.x() * rpyd.y() * tan_pitch
              + (angacc.x() + angvel.y() * rpyd.z()) * cos_yaw / cos_pitch
              + (angacc.y() - angvel.x() * rpyd.z()) * sin_yaw / cos_pitch;
  rpydd.y() =
    (angacc.y() - angvel.x() * rpyd.z()) * cos_yaw - (angacc.x() + angvel.y() * rpyd.z()) * sin_yaw;
  rpydd.z() = angacc.z()
              + (angvel.x() + cos_yaw + angvel.y() * sin_yaw) * rpyd.y() / dh_std::sqr(cos_pitch)
              + (angacc.x() + angvel.y() * rpyd.z()) * cos_yaw * tan_pitch
              + (angacc.y() - angvel.x() * rpyd.z()) * sin_yaw * tan_pitch;

  return rpydd;
}

Vector3d angaccFromEuleraccLocal(
  const double& roll,
  const double& pitch,
  const double& droll,
  const double& dpitch,
  const double& dyaw,
  const double& ddroll,
  const double& ddpitch,
  const double& ddyaw)
{
  const double cos_roll = cos(roll);
  const double sin_roll = sin(roll);
  const double cos_pitch = cos(pitch);
  const double sin_pitch = sin(pitch);

  Vector3d dgyro;
  dgyro.x() = ddroll - ddyaw * sin_pitch - dpitch * dyaw * cos_pitch;
  dgyro.y() = ddpitch * cos_roll - droll * dpitch * sin_roll + ddyaw * sin_roll * cos_pitch
              + droll * dyaw * cos_roll * cos_pitch - dpitch * dyaw * sin_roll * sin_pitch;
  dgyro.z() = -ddpitch * sin_roll - droll * dpitch * cos_roll + ddyaw * cos_roll * cos_pitch
              - droll * dyaw * sin_roll * cos_pitch - dpitch * dyaw * cos_roll * sin_pitch;

  return dgyro;
}

Vector3d angaccFromEuleraccLocal(
  const double& roll,
  const double& pitch,
  const Vector3d& drpy,
  const Vector3d& ddrpy)
{
  return angaccFromEuleraccLocal(
    roll, pitch, drpy.x(), drpy.y(), drpy.z(), ddrpy.x(), ddrpy.y(), ddrpy.z());
}
}  // namespace eigen_tools
