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

double clampTargetRotSpeedAndWarn(
  const Drone& drone,
  const size_t& rotor_idx,
  const double& battery_voltage,
  const double& tar_speed)
{
  const auto min_speed = drone.minRotSpeed(rotor_idx, battery_voltage);
  const auto max_speed = drone.maxRotSpeed(rotor_idx, battery_voltage);

  if (tar_speed < min_speed - kRotSpeedMargin)
  {
    ROS_WARN_STREAM(
      "Target rotation speed of CH" << rotor_idx << " is too low: " << tar_speed << " < "
                                    << min_speed << " [rad/s]");
    return min_speed;
  }
  else if (tar_speed > max_speed + kRotSpeedMargin)
  {
    ROS_WARN_STREAM(
      "Target rotation speed of CH" << rotor_idx << " is too high: " << tar_speed << " > "
                                    << max_speed << " [rad/s]");
    return max_speed;
  }
  else
  {
    return tar_speed;
  }
}
}  // namespace tobas
