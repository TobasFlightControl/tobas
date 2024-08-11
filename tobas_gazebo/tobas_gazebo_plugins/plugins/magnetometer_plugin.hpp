#pragma once

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/random.hpp"

namespace gazebo
{
class GazeboMagnetometerPlugin : public SensorPlugin
{

  using self = GazeboMagnetometerPlugin;


public:
  explicit GazeboMagnetometerPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  rclcpp::Node::SharedPtr node_;

  // SDF parameters
  std::string link_name_;
  math::Vector3d offset_;                      // [m] B_Pos_BS
  double lat_0_;                           // [deg] 原点の北緯
  double lon_0_;                           // [deg] 原点の東経
  double alt_0_;                           // [m] 原点の高度
  math::Vector3d noise_normal_;                // [nT]
  math::Vector3d noise_uniform_initial_bias_;  // [nT]

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  gz::math::Vector3d init_bias_;  // [nT] 世界座標系の地磁気に加わるバイアス
  double lat_, lon_;                    // [deg] 現在位置の経緯度

  std::random_device rnd_dev_;
  NormalDistribution3dPtr noise_;

  PublisherPtr<> mag_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void onUpdate();
  void publishMagMsg(const gz::math::Vector3d& field) const;
};
}  // namespace gazebo
