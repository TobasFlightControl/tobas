#pragma once

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
class GazeboBarometerPlugin : public SensorPlugin
{
  // Constants
  static constexpr char kPluginName[] = "barometer_plugin";

  // Default values
  static constexpr double kDefaultPressureVar = 1.;  // [Pa]

  using super = SensorPlugin;
  using PressureMsg = sensor_msgs::msg::FluidPressure;

public:
  explicit GazeboBarometerPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  rclcpp::NodeHandle node_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  SdfVector3 offset_;  // B_Pos_BS
  double alt_0_;
  double pressure_var_;

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  NormalDistribution pressure_noise_;
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  PublisherPtr<> pressure_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
};
}  // namespace gazebo
