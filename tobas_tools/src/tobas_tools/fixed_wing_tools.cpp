#include <dh_std_tools/math.hpp>

#include "../../include/tobas_tools/fixed_wing_tools.hpp"

#define MIN_AIR_SPEED_THRESH 0.1  // 最小風速 [m/s]

using namespace std;
using namespace Eigen;

double angleOfAttack(double u, double v, double w)
{
  return u > MIN_AIR_SPEED_THRESH ? atan(w / u) : 0.;
}

double angleOfAttack(const KDL::Vector& linvel_B)
{
  return angleOfAttack(linvel_B.x(), linvel_B.y(), linvel_B.z());
}

double angleOfSideSlip(double u, double v, double w)
{
  double V = dh_std::norm(u, v, w);
  return V > MIN_AIR_SPEED_THRESH ? asin(v / V) : 0.;
}

double angleOfSideSlip(const KDL::Vector& linvel_B)
{
  return angleOfSideSlip(linvel_B.x(), linvel_B.y(), linvel_B.z());
}
