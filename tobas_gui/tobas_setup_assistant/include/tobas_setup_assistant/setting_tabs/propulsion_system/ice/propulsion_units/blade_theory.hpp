#pragma once

#include <tobas_math/definitions.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
/* Unsteady Aerodynamic Parameter Estimation for Multirotor Helicopters [Nguyen+, 2019] */
class BladeTheory
{
  static constexpr double a = M_2PI;    // 2D lift curve slope (ideal value)
  static constexpr double B = 0.9;      // Tip loss factor
  static constexpr double gamma = 8.0;  // Lock number (typical value, cf. Balic Helicopter Aerodynamics p.66)
  static constexpr double C_d0 = 0.02;  // Profile drag coefficient (typical value)

public:
  explicit BladeTheory(
    int num_blades,
    double radius,
    double blade_chord,
    double pitch_angle,
    double air_density = tobas_std::kStandardAirDensity);

  std::pair<double, double> motorConst() const;
  std::pair<double, double> momentConst() const;
  std::pair<double, double> dragConst() const;

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

  /* dlam / dtheta */
  double lambdaDeriv() const;
};
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
