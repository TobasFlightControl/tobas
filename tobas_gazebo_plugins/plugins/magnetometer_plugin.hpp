#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/MagneticField.h>

#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/random.hpp"

namespace gazebo
{
// Constants
static constexpr char kPluginName[] = "magnetometer_plugin";

class GazeboMagnetometerPlugin : public SensorPlugin
{
  using super = SensorPlugin;

  using MagMsg = sensor_msgs::MagneticField;

public:
  explicit GazeboMagnetometerPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  SdfVector3 offset_;                      // [m] B_Pos_BS
  double lat_0_;                           // [deg] 原点の北緯
  double lon_0_;                           // [deg] 原点の東経
  double alt_0_;                           // [m] 原点の高度
  SdfVector3 noise_normal_;                // [nT]
  SdfVector3 noise_uniform_initial_bias_;  // [nT]

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  ignition::math::Vector3d init_bias_;  // [nT] 世界座標系の地磁気に加わるバイアス
  double lat_, lon_;                    // [deg] 現在位置の経緯度
  MagMsg mag_msg_;

  std::random_device rnd_dev_;
  NormalDistribution3dPtr noise_;

  ros::Publisher mag_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
};
}  // namespace gazebo
