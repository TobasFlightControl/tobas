#include "tobas_setup_assistant/setting_tabs/propulsion_system/electric/propulsion_units/aerodynamics/util.hpp"

#include <tobas_math/core.hpp>
#include <tobas_math/definitions.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace electric
{
namespace
{
/* 比率の最小2乗解 (memo: 2-28) */
double ratioLeastSquare(const Eigen::VectorXd& num, const Eigen::VectorXd& den)
{
  assert(num.size() == den.size());
  return num.dot(den) / den.squaredNorm();
}
}  // namespace

double motorConstFromThrustStand(const Eigen::VectorXd& rpms, const Eigen::VectorXd& thrusts)
{
  const auto omega2 = (rpms * tbs::kRpmToRps).cwiseAbs2();
  return ratioLeastSquare(thrusts, omega2);
}

double momentConstFromThrustStand(const Eigen::VectorXd& thrusts, const Eigen::VectorXd& torques)
{
  return ratioLeastSquare(torques, thrusts);
}

double motorConstFromUiuc(const Eigen::VectorXd& cts, double d)
{
  const auto ct = cts.mean();
  constexpr auto rho = tbs::kStandardAirDensity;  // TODO: ランタイムの気圧変化を考慮
  return (ct * rho * math::quar(d)) / math::sqr(M_2PI);
}

double momentConstFromUiuc(const Eigen::VectorXd& cts, const Eigen::VectorXd& cps, double d)
{
  const auto num = d * cps;
  const auto den = M_2PI * cts;
  return ratioLeastSquare(num, den);
}
}  // namespace electric
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
