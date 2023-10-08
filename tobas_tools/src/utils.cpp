#include <chrono>

#include <dh_std_tools/time.hpp>
#include <dh_std_tools/iostream.hpp>
#include <dh_kdl/kdl_parser.hpp>
#include <dh_kdl/treejnttoinertiasolver.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_tools/utils.hpp"

using namespace std;
using namespace KDL;

namespace tobas
{
double getMass()
{
  KDL::Tree tree;
  if (!treeFromParam(ros::this_node::getNamespace() + "/robot_description", tree))
  {
    throw runtime_error("Failed to get KDL tree.");
  }

  KDL::TreeJntToInertiaSolver inertia_solver_(tree);
  return inertia_solver_.JntToMass();
}

geomag::Elements geomag(const double& lat, const double& lon, const double& height)
{
  const auto year_frac = dh_std::yearFraction();
  if (year_frac < 2020)
  {
    throw runtime_error("Invalid year: " + to_string(year_frac));
  }

  // 5年ごとに新しいデータが出るので，それを過ぎたら警告する
  // World Magnetic Model: https://www.ncei.noaa.gov/products/world-magnetic-model
  if (year_frac - 2020 > 5)
  {
    dh_std::warn("It is time to replace the WMM data with the latest version.");
  }

  const auto position = geomag::geodetic2ecef(lat, lon, height);
  const auto mag_field = geomag::GeoMag(year_frac, position, geomag::WMM2020);
  return geomag::magField2Elements(mag_field, lat, lon);
}
}  // namespace tobas
