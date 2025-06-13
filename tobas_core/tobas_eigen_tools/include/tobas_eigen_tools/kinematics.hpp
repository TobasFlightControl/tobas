#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace eigen
{
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
