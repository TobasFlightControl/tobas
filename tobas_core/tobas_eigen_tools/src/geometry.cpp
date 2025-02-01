#include <tobas_math/core.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_std_tools/assert.hpp>

#include "../include/tobas_eigen_tools/geometry.hpp"

#define EPS numeric_limits<double>::epsilon()

using namespace std;
using namespace Eigen;

namespace eigen
{
void quaternionFromAccelMag(const Vector3d& a, const Vector3d& m, const Vector3d& m0, Quaterniond& q)
{
  tobas_std::quaternionFromAccelMag(
    a.x(), a.y(), a.z(), m.x(), m.y(), m.z(), m0.x(), m0.y(), m0.z(), q.x(), q.y(), q.z(), q.w());
}

Matrix3d angvelFromEulerrateGlobal(double pitch, double yaw)
{
  const auto cos_pitch = cos(pitch);
  const auto sin_pitch = sin(pitch);
  const auto cos_yaw = cos(yaw);
  const auto sin_yaw = sin(yaw);

  Matrix3d res;
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

Vector3d angvelFromEulerrateGlobal(const Vector3d& rpyd, double pitch, double yaw)
{
  return angvelFromEulerrateGlobal(pitch, yaw) * rpyd;
}

Matrix3d angvelFromEulerrateLocal(double roll, double pitch)
{
  const auto cos_roll = cos(roll);
  const auto sin_roll = sin(roll);
  const auto cos_pitch = cos(pitch);
  const auto sin_pitch = sin(pitch);

  Matrix3d res;
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

Vector3d angvelFromEulerrateLocal(const Vector3d& rpyd, double roll, double pitch)
{
  return angvelFromEulerrateLocal(roll, pitch) * rpyd;
}

Matrix3d eulerrateFromAngvelGlobal(double pitch, double yaw)
{
  const auto cos_pitch = cos(pitch);
  const auto tan_pitch = tan(pitch);
  const auto cos_yaw = cos(yaw);
  const auto sin_yaw = sin(yaw);
  assert(cos_pitch > EPS);

  Matrix3d res;
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

Vector3d eulerrateFromAngvelGlobal(const Vector3d& angvel, double pitch, double yaw)
{
  return eulerrateFromAngvelGlobal(pitch, yaw) * angvel;
}

Matrix3d eulerrateFromAngvelLocal(double roll, double pitch)
{
  const auto cos_roll = cos(roll);
  const auto sin_roll = sin(roll);
  const auto cos_pitch = cos(pitch);
  const auto tan_pitch = tan(pitch);
  assertWithMsg(cos_pitch > EPS, "roll: " << roll << ", pitch: " << pitch);

  Matrix3d res;
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

Vector3d eulerrateFromAngvelLocal(const Vector3d& angvel, double roll, double pitch)
{
  return eulerrateFromAngvelLocal(roll, pitch) * angvel;
}

Vector3d euleraccFromAngaccGlobal(const Vector3d& angvel, const Vector3d& angacc, double pitch, double yaw)
{
  const auto cos_pitch = cos(pitch);
  const auto tan_pitch = tan(pitch);
  const auto cos_yaw = cos(yaw);
  const auto sin_yaw = sin(yaw);

  const auto rpyd = eulerrateFromAngvelGlobal(angvel, pitch, yaw);

  Vector3d rpydd;
  rpydd.x() = rpyd.x() * rpyd.y() * tan_pitch + (angacc.x() + angvel.y() * rpyd.z()) * cos_yaw / cos_pitch
              + (angacc.y() - angvel.x() * rpyd.z()) * sin_yaw / cos_pitch;
  rpydd.y() = (angacc.y() - angvel.x() * rpyd.z()) * cos_yaw - (angacc.x() + angvel.y() * rpyd.z()) * sin_yaw;
  rpydd.z() = angacc.z() + (angvel.x() + cos_yaw + angvel.y() * sin_yaw) * rpyd.y() / math::sqr(cos_pitch)
              + (angacc.x() + angvel.y() * rpyd.z()) * cos_yaw * tan_pitch
              + (angacc.y() - angvel.x() * rpyd.z()) * sin_yaw * tan_pitch;

  return rpydd;
}

Vector3d angaccFromEuleraccLocal(
  double roll,
  double pitch,
  double droll,
  double dpitch,
  double dyaw,
  double ddroll,
  double ddpitch,
  double ddyaw)
{
  const auto cos_roll = cos(roll);
  const auto sin_roll = sin(roll);
  const auto cos_pitch = cos(pitch);
  const auto sin_pitch = sin(pitch);

  Vector3d dgyro;
  dgyro.x() = ddroll - ddyaw * sin_pitch - dpitch * dyaw * cos_pitch;
  dgyro.y() = ddpitch * cos_roll - droll * dpitch * sin_roll + ddyaw * sin_roll * cos_pitch
              + droll * dyaw * cos_roll * cos_pitch - dpitch * dyaw * sin_roll * sin_pitch;
  dgyro.z() = -ddpitch * sin_roll - droll * dpitch * cos_roll + ddyaw * cos_roll * cos_pitch
              - droll * dyaw * sin_roll * cos_pitch - dpitch * dyaw * cos_roll * sin_pitch;

  return dgyro;
}

Vector3d angaccFromEuleraccLocal(double roll, double pitch, const Vector3d& drpy, const Vector3d& ddrpy)
{
  return angaccFromEuleraccLocal(roll, pitch, drpy.x(), drpy.y(), drpy.z(), ddrpy.x(), ddrpy.y(), ddrpy.z());
}
}  // namespace eigen
