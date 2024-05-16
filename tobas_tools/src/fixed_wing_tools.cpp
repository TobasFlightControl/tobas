#include <tobas_std_tools/math.hpp>

#include "../include/tobas_tools/fixed_wing_tools.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas
{
double angleOfAttack(const double& u, const double& w)
{
  return u > kMinAirSpeedThresh ? atan(w / u) : 0.;
}

double angleOfAttack(const Vector& linvel_B)
{
  return angleOfAttack(linvel_B.x(), linvel_B.z());
}

double angleOfSideSlip(const double& u, const double& v, const double& w)
{
  const double V = tobas_std::norm(u, v, w);
  return V > kMinAirSpeedThresh ? asin(v / V) : 0.;
}

double angleOfSideSlip(const Vector& linvel_B)
{
  return angleOfSideSlip(linvel_B.x(), linvel_B.y(), linvel_B.z());
}

double dynamicPressure(const double& rho, const double& V)
{
  assert(rho > 0.);
  assert(V >= 0.);
  return rho * tobas_std::sqr(V) / 2.;
}
}  // namespace tobas
