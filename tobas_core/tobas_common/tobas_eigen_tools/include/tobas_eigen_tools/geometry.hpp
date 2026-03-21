#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tobas_math/float.hpp>

namespace eigen
{
/* 3次元ベクトルを FLU (Front-Left-Up) 座標系から FRD (Front-Right-Down) 座標系に変換する．Rx(π)をかけるのと同じ． */
inline void vectorFrdToFlu(const Eigen::Vector3d& src, Eigen::Vector3d& dst)
{
  dst.x() = src.x();
  dst.y() = -src.y();
  dst.z() = -src.z();
}

inline void vectorFluToFrd(const Eigen::Vector3d& src, Eigen::Vector3d& dst)
{
  vectorFrdToFlu(src, dst);
}

inline void vectorFrdToFlu(Eigen::Vector3d& arg)
{
  vectorFrdToFlu(arg, arg);
}

inline void vectorFluToFrd(Eigen::Vector3d& arg)
{
  vectorFluToFrd(arg, arg);
}

inline Eigen::AngleAxisd angleAxisFromVector(const Eigen::Vector3d& w)
{
  const auto angle = w.norm();
  const auto axis = (angle == 0) ? Eigen::Vector3d::UnitX() : w.normalized();
  return Eigen::AngleAxisd(angle, axis);
}

inline Eigen::Vector3d vectorFromAngleAxis(const Eigen::AngleAxisd& angle_axis)
{
  return angle_axis.angle() * angle_axis.axis();
}

/* 等価角軸ベクトルから回転行列を作成． */
inline Eigen::Matrix3d dcmFromAngleAxis(const Eigen::Vector3d& w)
{
  return angleAxisFromVector(w).toRotationMatrix();
}

/* 等価角軸ベクトルからクオータニオンを作成． */
inline Eigen::Quaterniond quaternionFromAngleAxis(const Eigen::Vector3d& w)
{
  return Eigen::Quaterniond(angleAxisFromVector(w));
}

/* クオータニオンから等価角軸ベクトルを作成． */
inline Eigen::Vector3d angleAxisFromQuaternion(const Eigen::Quaterniond& q)
{
  Eigen::AngleAxisd angle_axis(q);
  return vectorFromAngleAxis(angle_axis);
}

inline Eigen::Quaterniond quaternionFromRPY(double roll, double pitch, double yaw)
{
  const Eigen::AngleAxisd rot_yaw(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd rot_pitch(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rot_roll(roll, Eigen::Vector3d::UnitX());
  return rot_yaw * rot_pitch * rot_roll;
}

inline Eigen::Matrix3d dcmFromRPY(double roll, double pitch, double yaw)
{
  return quaternionFromRPY(roll, pitch, yaw).toRotationMatrix();
}

inline double yawFromDCM(const Eigen::Matrix3d& R)
{
  return atan2(R(1, 0), R(0, 0));
}

/* ハミルトン(w,x,y,z)をQuaterniondに変換． */
inline Eigen::Quaterniond quaternionFromHamilton(const Eigen::Vector4d& ham)
{
  return Eigen::Quaterniond((Eigen::Vector4d() << ham.tail<3>(), ham.head<1>()).finished());
}

/* Quaterniondをハミルトン(w,x,y,z)に変換． */
inline Eigen::Vector4d hamiltonFromQuaternion(const Eigen::Quaterniond& q)
{
  // Quaternionの要素はメモリ上で連続しているとは限らないため，coeffsを呼ぶとコンパイルエラーになる恐れがある．
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

/* ベクトルの外積に相当する行列を作成する． */
inline Eigen::Matrix3d skew(double x, double y, double z)
{
  return (Eigen::Matrix3d() << 0, -z, y, z, 0, -x, -y, x, 0).finished();
}

/* ベクトルの外積に相当する行列を作成する． */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  return skew(v(0), v(1), v(2));
}

/* ベクトルの外積に相当する行列の2乗を作成する． */
inline Eigen::Matrix3d skew2(double x, double y, double z)
{
  const auto xx = x * x;
  const auto yy = y * y;
  const auto zz = z * z;
  const auto xy = x * y;
  const auto yz = y * z;
  const auto zx = z * x;

  return (Eigen::Matrix3d() << -(yy + zz), xy, zx, xy, -(zz + xx), yz, zx, yz, -(xx + yy)).finished();
}

/* ベクトルの外積に相当する行列の2乗を作成する． */
inline Eigen::Matrix3d skew2(const Eigen::Vector3d& v)
{
  return skew2(v(0), v(1), v(2));
}

/* x0を通る方向v (||v|| = 1) の直線に対しpから下ろした垂点を求める． */
inline Eigen::Vector3d
projectPointOnToLine(const Eigen::Vector3d& x0, const Eigen::Vector3d& v, const Eigen::Vector3d& p)
{
  assert(math::isClose(v.norm(), 1.));
  return x0 + v.dot(p - x0) * v;
}
}  // namespace eigen
