#pragma once

#include <cmath>

namespace tobas_std
{
/* ===== 角度の単位の変換．引数が整数の時に0が返るのを避けるためテンプレートにはしない． ===== */

/* degree -> radian */
constexpr inline double deg2rad(const double& deg)
{
  return deg * (M_PI / 180.);
}

/* radian -> degree */
constexpr inline double rad2deg(const double& rad)
{
  return rad * (180. / M_PI);
}

/* rpm -> rad/s */
constexpr inline double rpm2rps(const double& rpm)
{
  return rpm * (M_PI / 30.);
}

/* rad/s -> rpm */
constexpr inline double rps2rpm(const double& rps)
{
  return rps * (30. / M_PI);
}

/* ========== */
}  // namespace tobas_std
