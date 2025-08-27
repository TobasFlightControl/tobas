#pragma once

#include <cmath>

namespace tobas_std
{
/* degree -> radian */
inline constexpr double deg2rad(const double& deg)
{
  return deg * (M_PI / 180.);
}

/* radian -> degree */
inline constexpr double rad2deg(const double& rad)
{
  return rad * (180. / M_PI);
}

/* rpm -> rad/s */
inline constexpr double rpm2rps(const double& rpm)
{
  return rpm * (M_PI / 30.);
}

/* rad/s -> rpm */
inline constexpr double rps2rpm(const double& rps)
{
  return rps * (30. / M_PI);
}

/* inch -> m */
inline constexpr double inch2meter(const double& inch)
{
  return inch * 0.0254;
}

/* m -> inch */
inline constexpr double meter2inch(const double& meter)
{
  return meter / 0.0254;
}

/* feet -> m */
inline constexpr double feet2meter(const double& feet)
{
  return feet * 0.3048;
}

/* m -> feet */
inline constexpr double meter2feet(const double& meter)
{
  return meter / 0.3048;
}

/* yard -> m */
inline constexpr double yard2meter(const double& yard)
{
  return yard * 0.9144;
}

/* m -> yard */
inline constexpr double meter2yard(const double& meter)
{
  return meter / 0.9144;
}
}  // namespace tobas_std
