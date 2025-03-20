#include <tobas_math/core.hpp>
#include <tobas_std_tools/check.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/blade_theory.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
BladeTheory::BladeTheory(int num_blades, double radius, double blade_chord, double pitch_angle, double air_density)
  : N_(num_blades), R_(radius), c_(blade_chord), theta_(pitch_angle), rho_(air_density)
{
  TOBAS_CHECK(num_blades > 0);
  TOBAS_CHECK(radius > 0.);
  TOBAS_CHECK(blade_chord > 0.);
  TOBAS_CHECK(0. < pitch_angle && pitch_angle < M_PI_2);
  TOBAS_CHECK(air_density > 0.);
}

double BladeTheory::motorConst() const
{
  return 4 * M_PI * rho_ * math::quat(R_) * C_T();
}

double BladeTheory::momentConst() const
{
  return R_ * lambda();
}

double BladeTheory::dragConst() const
{
  return 4 * M_PI * rho_ * math::cube(R_) * C_H();
}

double BladeTheory::sigma() const
{
  return (N_ * c_) / (M_PI * R_);
}

double BladeTheory::lambda() const
{
  const auto a_B_sigma = a * B * sigma();
  return a_B_sigma * B / 16 * (sqrt(1 + (64 * theta_) / (3 * a_B_sigma)) - 1);
}

double BladeTheory::C_T() const
{
  return 2 * math::sqr(lambda());
}

double BladeTheory::C_H() const
{
  const auto lam = lambda();
  const auto b0 = 0.5 * gamma * (theta_ / 4 - lam / 3);
  const auto b1c = 2 * (lam - (4. / 3) * theta_);  // devided by mu
  const auto b1s = -(4. / 3) * b0;                 // devided by mu
  return (sigma() / 4)
         * (C_d0 + (a / 6) * (2 * theta_ * (3 * lam - 2 * b1c) + 9 * lam * b1c + 2 * b0 * b1s + 3 * math::sqr(b0)));
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
