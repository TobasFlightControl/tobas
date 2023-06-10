#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/NavSatFix.h>

#include <tobas_msgs/LinearVelocityWithCovariance.h>

#include "../tobas_gazebo_plugins/common.hpp"
#include "../tobas_gazebo_plugins/random.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "gps_plugin";

// Default values
static const std::string kDefaultGpsTopic = "gps";
static const std::string kDefaultGroundSpeedTopic = "ground_speed";
static constexpr double kDefaultHorPosStdDev = 3.;
static constexpr double kDefaultVerPosStdDev = 6.;
static constexpr double kDefaultHorVelStdDev = 0.1;
static constexpr double kDefaultVerVelStdDev = 0.1;

/**
 * @brief GPSの位置データと速度データを発行するプラグイン．
 *
 * @note 更新頻度を設定するためにModelPluginではなくSensorPluginを継承している．
 */
class GazeboGpsPlugin : public SensorPlugin
{
  using super = SensorPlugin;

  using PosMsg = sensor_msgs::NavSatFix;
  using VelMsg = tobas_msgs::LinearVelocityWithCovariance;

public:
  explicit GazeboGpsPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string gps_topic_;
  std::string vel_topic_;
  double hor_pos_std_dev_;
  double ver_pos_std_dev_;
  double hor_vel_std_dev_;
  double ver_vel_std_dev_;
  double lat_0_;  // 原点の北緯
  double lon_0_;  // 原点の東経
  double alt_0_;  // 原点の高度

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  PosMsg pos_msg_;
  VelMsg vel_msg_;

  std::random_device rnd_dev_;
  NormalDistribution3dPtr pos_noise_;
  NormalDistribution3dPtr vel_noise_;

  // Publishers
  ros::Publisher pos_pub_;
  ros::Publisher vel_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void fillMessageStaticParts();
  void setRandomDistribuitons();
  void registerPublishers();
  void onUpdate();
  void updatePosition();
  void updateVelocity();
};
}  // namespace gazebo
