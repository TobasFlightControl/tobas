#pragma once

#include <cmath>

namespace tobas_std
{
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

/* inch -> m */
constexpr inline double inch2meter(const double& inch)
{
  return inch * 0.0254;
}

/* m -> inch */
constexpr inline double meter2inch(const double& meter)
{
  return meter / 0.0254;
}

/* feet -> m */
constexpr inline double feet2meter(const double& feet)
{
  return feet * 0.3048;
}

/* m -> feet */
constexpr inline double meter2feet(const double& meter)
{
  return meter / 0.3048;
}

/* yard -> m */
constexpr inline double yard2meter(const double& yard)
{
  return yard * 0.9144;
}

/* m -> yard */
constexpr inline double meter2yard(const double& meter)
{
  return meter / 0.9144;
}
}  // namespace tobas_std
