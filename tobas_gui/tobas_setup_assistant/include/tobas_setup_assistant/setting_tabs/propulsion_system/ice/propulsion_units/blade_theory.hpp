#pragma once

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/aerodynamics.hpp>
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
/* 可変ピッチプロペラの空力パラメータをブレード理論とテイラー展開で近似する (memo: 3-36) */
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
    double air_density = tbs::kStandardAirDensity);

  tobas::VppMotorConstant motorConst() const;
  tobas::VppMomentConstant momentConst() const;
  tobas::VppDragConstant dragConst() const;

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
