#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace eigen_tools
{
/* 3次元ベクトルをNED座標系からNWU座標系に変換する．Rx(π)をかけるのと同じ． */
void vectorNedToNwu(const Eigen::Vector3d& src, Eigen::Vector3d& des);
void vectorNwuToNed(const Eigen::Vector3d& src, Eigen::Vector3d& des);
void vectorNedToNwu(Eigen::Vector3d& arg);
void vectorNwuToNed(Eigen::Vector3d& arg);

Eigen::AngleAxisd vectorToAngleAxis(const Eigen::Vector3d& w);
Eigen::Vector3d angleAxisToVector(const Eigen::AngleAxisd& angle_axis);

/* 等価角軸ベクトルから回転行列を作成． */
Eigen::Matrix3d angleAxisToRotMat(const Eigen::Vector3d& w);
/* 等価角軸ベクトルからクオータニオンを作成． */
Eigen::Quaterniond angleAxisToQuaternion(const Eigen::Vector3d& w);
/* クオータニオンから等価角軸ベクトルを作成． */
Eigen::Vector3d quaternionToAngleAxis(const Eigen::Quaterniond& q);

Eigen::Quaterniond quaternionFromRPY(const double& roll, const double& pitch, const double& yaw);
Eigen::Matrix3d dcmFromRPY(const double& roll, const double& pitch, const double& yaw);

/* ハミルトン(w,x,y,z)をQuaterniondに変換． */
Eigen::Quaterniond hamiltonToQuaternion(const Eigen::Vector4d& ham);
/* Quaterniondをハミルトン(w,x,y,z)に変換． */
Eigen::Vector4d quaternionToHamilton(const Eigen::Quaterniond& q);

/* ベクトルの外積に相当する行列を作成する． */
Eigen::Matrix3d crossMat(const double& x, const double& y, const double& z);
/* ベクトルの外積に相当する行列を作成する． */
Eigen::Matrix3d crossMat(const Eigen::Vector3d& v);
/* ベクトルの外積に相当する行列の2乗を作成する． */
Eigen::Matrix3d crossMat2(const double& x, const double& y, const double& z);
/* ベクトルの外積に相当する行列の2乗を作成する． */
Eigen::Matrix3d crossMat2(const Eigen::Vector3d& v);

void imuToQuaternion(
  const Eigen::Vector3d& a,
  const Eigen::Vector3d& m,
  const Eigen::Vector3d& m0,
  Eigen::Quaterniond& q);

/* ZYXオイラー角の変化率をグローバル座標系で表現された角速度に変換する行列を返す． */
Eigen::Matrix3d angvelFromEulerrateGlobal(const double& pitch, const double& yaw);

/* ZYXオイラー角の変化率をグローバル座標系で表現された角速度に変換する． */
Eigen::Vector3d angvelFromEulerrateGlobal(const Eigen::Vector3d& eulerrate, const double& pitch, const double& yaw);

/* ZYXオイラー角の変化率をローカル座標系で表現された角速度に変換する行列を返す． */
Eigen::Matrix3d angvelFromEulerrateLocal(const double& roll, const double& pitch);

/* ZYXオイラー角の変化率をローカル座標系で表現された角速度に変換する． */
Eigen::Vector3d angvelFromEulerrateLocal(const Eigen::Vector3d& eulerrate, const double& roll, const double& pitch);

/* グローバル座標系で表現された角速度をZYXオイラー角の変化率に変換する行列を計算する． */
Eigen::Matrix3d eulerrateFromAngvelGlobal(const double& pitch, const double& yaw);

/* グローバル座標系で表現された角速度をZYXオイラー角の変化率に変換する． */
Eigen::Vector3d eulerrateFromAngvelGlobal(const Eigen::Vector3d& angvel, const double& pitch, const double& yaw);

/**
 * @brief ローカル座標系で表現された角速度をZYXオイラー角の変化率に変換する行列を計算する．
 * cf. https://www.sky-engin.jp/blog/eulerian-angles/#toc7
 */
Eigen::Matrix3d eulerrateFromAngvelLocal(const double& roll, const double& pitch);

/* ローカル座標系で表現された角速度をZYXオイラー角の変化率に変換する． */
Eigen::Vector3d eulerrateFromAngvelLocal(const Eigen::Vector3d& angvel, const double& roll, const double& pitch);

/* 角軸ベクトルを回転行列に変換する． */
Eigen::Matrix3d matrixFromAngleAxis(const Eigen::Vector3d& a);

/* 回転行列を角軸ベクトルに変換する． */
Eigen::Vector3d AngleAxisFromMatrix(const Eigen::Matrix3d& r);

/* グローバル座標系で表現された角加速度をオイラー角加速度に変換する． */
Eigen::Vector3d euleraccFromAngaccGlobal(
  const Eigen::Vector3d& angvel,
  const Eigen::Vector3d& angacc,
  const double& pitch,
  const double& yaw);

/* オイラー角加速度をローカル座標系で表現された角加速度に変換する (memo: 2-41)． */
Eigen::Vector3d angaccFromEuleraccLocal(
  const double& roll,
  const double& pitch,
  const double& droll,
  const double& dpitch,
  const double& dyaw,
  const double& ddroll,
  const double& ddpitch,
  const double& ddyaw);

/* オイラー角加速度をローカル座標系で表現された角加速度に変換する (memo: 2-41)． */
Eigen::Vector3d angaccFromEuleraccLocal(
  const double& roll,
  const double& pitch,
  const Eigen::Vector3d& drpy,
  const Eigen::Vector3d& ddrpy);
}  // namespace eigen_tools
