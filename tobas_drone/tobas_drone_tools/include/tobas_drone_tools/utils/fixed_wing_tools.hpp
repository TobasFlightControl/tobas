#pragma once

#include <eigen3/Eigen/Core>

#include <tobas_constants/constants.hpp>
#include <tobas_math/linalg.hpp>

namespace tobas
{
/**
 * @brief 迎角 (alpha) を計算する．
 *
 * @param u,w 風に対する相対的な機体速度 (NED座標系) [m/s]
 * @return double 迎角 [rad]
 */
inline double angleOfAttack(const double& u, const double& w)
{
  return u > kMinAirSpeedThresh ? atan(w / u) : 0;
}

/**
 * @brief 迎角 (alpha) を計算する．
 *
 * @param linvel_B 風に対する相対的な機体速度 (NED座標系) [m/s]
 * @return double 迎角 [rad]
 */
inline double angleOfAttack(const Eigen::Vector3d& linvel_B)
{
  return angleOfAttack(linvel_B.x(), linvel_B.z());
}

/**
 * @brief 横滑り角 (beta) を計算する．
 *
 * @param u,v,w 風に対する相対的な機体速度 (NED座標系) [m/s]
 * @return double 横滑り角 [rad]
 */
inline double angleOfSideSlip(const double& u, const double& v, const double& w)
{
  const auto V = math::norm(u, v, w);
  return V > kMinAirSpeedThresh ? asin(v / V) : 0;
}

/**
 * @brief 横滑り角 (beta) を計算する．
 *
 * @param linvel_B 風に対する相対的な機体速度 (NED座標系) [m/s]
 * @return double 横滑り角 [rad]
 */
inline double angleOfSideSlip(const Eigen::Vector3d& linvel_B)
{
  return angleOfSideSlip(linvel_B.x(), linvel_B.y(), linvel_B.z());
}

/**
 * @brief 動圧 (q_bar) を計算する．
 *
 * @param rho 大気密度 [kg/m^3]
 * @param V 風に対する相対的な機体速度の絶対値 [m/s]
 * @return double 動圧 [Pa]
 */
inline double dynamicPressure(const double& rho, const double& V)
{
  assert(rho > 0);
  assert(V >= 0);
  return rho * math::sqr(V) / 2;
}
}  // namespace tobas
