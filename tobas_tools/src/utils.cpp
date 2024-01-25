#include <tobas_std_tools/time.hpp>
#include <tobas_std_tools/console.hpp>
#include <tobas_kdl/kdl_parser.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>
#include <tobas_ros_tools/exception.hpp>

#include "../include/tobas_tools/utils.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas
{
double getMass()
{
  KDL::Tree tree;
  if (!treeFromParam(ros::this_node::getNamespace() + "/robot_description", tree))
    throw runtime_error("Failed to get KDL tree.");

  KDL::TreeJntToInertiaSolver inertia_solver(tree);
  if (inertia_solver.JntToCart(JntArray::Zero(tree.getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver.errorMessage());

  return inertia_solver.getInertia().getMass();
}

geomag::Elements geomag(const double& lat, const double& lon, const double& height)
{
  const auto year_frac = tobas_std::yearFraction();
  if (year_frac < 2020)
  {
    throw runtime_error("Invalid year: " + to_string(year_frac));
  }

  // 5年ごとに新しいデータが出るので，それを過ぎたら警告する
  // World Magnetic Model: https://www.ncei.noaa.gov/products/world-magnetic-model
  if (year_frac - 2020 > 5)
  {
    TOBAS_WARN("It is time to replace the WMM data with the latest version.");
  }

  const auto position = geomag::geodetic2ecef(lat, lon, height);
  const auto mag_field = geomag::GeoMag(year_frac, position, geomag::WMM2020);
  return geomag::magField2Elements(mag_field, lat, lon);
}
}  // namespace tobas
