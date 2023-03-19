#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <sensor_msgs/NavSatFix.h>

#include <multirotor_msgs/LinearVelocityWithCovarianceStamped.h>

namespace gazebo
{
// Default values
static constexpr char defaultGpsTopic[] = "gps";
static constexpr char defaultGroundSpeedTopic[] = "ground_speed";
static constexpr double defaultHorPosStdDev = 3.;
static constexpr double defaultVerPosStdDev = 6.;
static constexpr double defaultHorVelStdDev = 0.1;
static constexpr double defaultVerVelStdDev = 0.1;
static constexpr double defaultLatitudeZero = 35.658099;    // 日本: 北緯35度39分29秒
static constexpr double defaultLongitudeZero = 139.741354;  // 日本: 東経139度44分28秒8759

/**
 * @brief GPSの位置データと速度データを発行するプラグイン．
 *
 * @note 更新頻度を設定するためにModelPluginではなくSensorPluginを継承している．
 */
class GazeboGpsPlugin : public SensorPlugin
{
  using NormalDistribution = std::normal_distribution<double>;
  using PosMsg = sensor_msgs::NavSatFix;
  using VelMsg = multirotor_msgs::LinearVelocityWithCovarianceStamped;

public:
  GazeboGpsPlugin();

protected:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);
  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate();
  void updatePosition();
  void updateVelocity();

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

  physics::WorldPtr world_;                 // Pointer to the world
  physics::LinkPtr link_;                   // Pointer to the sensor link
  event::ConnectionPtr update_connection_;  // Pointer to the update event connection
  PosMsg pos_msg_;                          // GPS message to be published on sensor update
  VelMsg vel_msg_;                          // Ground speed message to be published on sensor update

  NormalDistribution pos_noise_[3];  // Normal distributions for GPS noise
  NormalDistribution vel_noise_[3];  // Normal distributions for ground speed noise
  std::random_device rnd_dev_;       // Random device
  std::mt19937 rnd_gen_;             // Random number generator

  // Publishers
  ros::Publisher pos_pub_;
  ros::Publisher vel_pub_;
};
}  // namespace gazebo
