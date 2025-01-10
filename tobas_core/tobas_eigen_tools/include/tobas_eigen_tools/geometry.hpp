#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace eigen
{
/* 3次元ベクトルをNED座標系からNWU座標系に変換する．Rx(π)をかけるのと同じ． */
inline void vectorNedToNwu(const Eigen::Vector3d& src, Eigen::Vector3d& des)
{
  des.x() = src.x();
  des.y() = -src.y();
  des.z() = -src.z();
}

inline void vectorNwuToNed(const Eigen::Vector3d& src, Eigen::Vector3d& des)
{
  vectorNedToNwu(src, des);
}

inline void vectorNedToNwu(Eigen::Vector3d& arg)
{
  vectorNedToNwu(arg, arg);
}

inline void vectorNwuToNed(Eigen::Vector3d& arg)
{
  vectorNwuToNed(arg, arg);
}

inline Eigen::AngleAxisd vectorToAngleAxis(const Eigen::Vector3d& w)
{
  const auto angle = w.norm();
  const auto axis = (angle == 0) ? Eigen::Vector3d::UnitX() : w.normalized();
  return Eigen::AngleAxisd(angle, axis);
}

inline Eigen::Vector3d angleAxisToVector(const Eigen::AngleAxisd& angle_axis)
{
  return angle_axis.angle() * angle_axis.axis();
}

/* 等価角軸ベクトルから回転行列を作成． */
inline Eigen::Matrix3d angleAxisToRotMat(const Eigen::Vector3d& w)
{
  return vectorToAngleAxis(w).toRotationMatrix();
}

/* 等価角軸ベクトルからクオータニオンを作成． */
inline Eigen::Quaterniond angleAxisToQuaternion(const Eigen::Vector3d& w)
{
  return Eigen::Quaterniond(vectorToAngleAxis(w));
}

/* クオータニオンから等価角軸ベクトルを作成． */
inline Eigen::Vector3d quaternionToAngleAxis(const Eigen::Quaterniond& q)
{
  Eigen::AngleAxisd angle_axis(q);
  return angleAxisToVector(angle_axis);
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

/* ハミルトン(w,x,y,z)をQuaterniondに変換． */
inline Eigen::Quaterniond hamiltonToQuaternion(const Eigen::Vector4d& ham)
{
  return Eigen::Quaterniond((Eigen::Vector4d() << ham.tail<3>(), ham.head<1>()).finished());
}

/* Quaterniondをハミルトン(w,x,y,z)に変換． */
inline Eigen::Vector4d quaternionToHamilton(const Eigen::Quaterniond& q)
{
  return (Eigen::Vector4d() << q.coeffs().tail<1>(), q.coeffs().head<3>()).finished();
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

void imuToQuaternion(
  const Eigen::Vector3d& a,
  const Eigen::Vector3d& m,
  const Eigen::Vector3d& m0,
  Eigen::Quaterniond& q);

/* ZYXオイラー角の変化率をグローバル座標系で表現された角速度に変換する行列を返す． */
Eigen::Matrix3d angvelFromEulerrateGlobal(double pitch, double yaw);

/* ZYXオイラー角の変化率をグローバル座標系で表現された角速度に変換する． */
Eigen::Vector3d angvelFromEulerrateGlobal(const Eigen::Vector3d& eulerrate, double pitch, double yaw);

/* ZYXオイラー角の変化率をローカル座標系で表現された角速度に変換する行列を返す． */
Eigen::Matrix3d angvelFromEulerrateLocal(double roll, double pitch);

/* ZYXオイラー角の変化率をローカル座標系で表現された角速度に変換する． */
Eigen::Vector3d angvelFromEulerrateLocal(const Eigen::Vector3d& eulerrate, double roll, double pitch);

/* グローバル座標系で表現された角速度をZYXオイラー角の変化率に変換する行列を計算する． */
Eigen::Matrix3d eulerrateFromAngvelGlobal(double pitch, double yaw);

/* グローバル座標系で表現された角速度をZYXオイラー角の変化率に変換する． */
Eigen::Vector3d eulerrateFromAngvelGlobal(const Eigen::Vector3d& angvel, double pitch, double yaw);

/**
 * @brief ローカル座標系で表現された角速度をZYXオイラー角の変化率に変換する行列を計算する．
 * cf. https://www.sky-engin.jp/blog/eulerian-angles/#toc7
 */
Eigen::Matrix3d eulerrateFromAngvelLocal(double roll, double pitch);

/* ローカル座標系で表現された角速度をZYXオイラー角の変化率に変換する． */
Eigen::Vector3d eulerrateFromAngvelLocal(const Eigen::Vector3d& angvel, double roll, double pitch);

/* グローバル座標系で表現された角加速度をオイラー角加速度に変換する． */
Eigen::Vector3d
euleraccFromAngaccGlobal(const Eigen::Vector3d& angvel, const Eigen::Vector3d& angacc, double pitch, double yaw);

/* オイラー角加速度をローカル座標系で表現された角加速度に変換する (memo: 2-41)． */
Eigen::Vector3d angaccFromEuleraccLocal(
  double roll,
  double pitch,
  double droll,
  double dpitch,
  double dyaw,
  double ddroll,
  double ddpitch,
  double ddyaw);

/* オイラー角加速度をローカル座標系で表現された角加速度に変換する (memo: 2-41)． */
Eigen::Vector3d
angaccFromEuleraccLocal(double roll, double pitch, const Eigen::Vector3d& drpy, const Eigen::Vector3d& ddrpy);
}  // namespace eigen
