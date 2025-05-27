#pragma once

#include <cmath>

#include <tobas_std_tools/universal_constants.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
/* Unsteady Aerodynamic Parameter Estimation for Multirotor Helicopters [Nguyen+, 2019] */
class BladeTheory
{
  static constexpr double a = 2 * M_PI;  // 2D lift curve slope (ideal value)
  static constexpr double B = 0.9;       // Tip loss factor
  static constexpr double gamma = 8.0;   // Lock number (typical value, cf. Balic Helicopter Aerodynamics p.66)
  static constexpr double C_d0 = 0.02;   // Profile drag coefficient (typical value)

public:
  explicit BladeTheory(
    int num_blades,
    double radius,
    double blade_chord,
    double pitch_angle,
    double air_density = tobas_std::kStandardAirDensity);

  double motorConst() const;
  double momentConst() const;
  double dragConst() const;

private:
  const int N_;
  const double R_;
  const double c_;
  const double theta_;
  const double rho_;

  /* Solidity */
  double sigma() const;

  /* Inflow ratio */
  double lambda() const;

  /* Thrust coefficient */
  double C_T() const;

  /* Horizontal force coefficient (divided by mu) */
  double C_H() const;
};
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
