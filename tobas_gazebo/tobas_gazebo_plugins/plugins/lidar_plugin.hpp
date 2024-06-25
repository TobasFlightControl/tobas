#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/plugins/RayPlugin.hh>

namespace gazebo
{
/**
 * @brief 3D LiDAR Plugin.
 * https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_block_laser.h
 */
class GazeboLidarPlugin : public RayPlugin
{
  // Constants
  static constexpr char kPluginName[] = "lidar_plugin";
  static constexpr double kEpsilonDiff = 1e-6;
  static constexpr double kTimeout = 1e-2;

  // Default values
  const std::string kDefaultFrameName = "world";
  static constexpr double kDefaultNoiseStddev = 0.;

  using super = RayPlugin;

public:
  explicit GazeboLidarPlugin();
  ~GazeboLidarPlugin();

  void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;
  void onStats(const boost::shared_ptr<msgs::WorldStatistics const>& msg);

protected:
  void OnNewLaserScans() override;

private:
  ros::NodeHandle nh_;  // pointer to ros node

  // SDF parameters
  std::string ns_;
  std::string frame_name_;  // frame transform name, should match link name
  double noise_stddev_;     // Gaussian noise

  sensors::SensorPtr parent_sensor_;  // The parent sensor
  sensors::RaySensorPtr parent_ray_sensor_;
  physics::MultiRayShapePtr shape_;

  size_t laser_connect_count_ = 0;  // Keep track of number of connctions
  common::Time last_update_time_;
  common::Time sim_time_;

  boost::mutex lock_;               // A mutex to lock access to fields that are used in message callbacks
  ros::CallbackQueue laser_queue_;  // Custom Callback Queue
  boost::thread callback_laser_queue_thread_;

  ros::Publisher pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void putLaserData(const common::Time& update_time);
  void laserConnect();
  void laserDisconnect();
  double gaussianKernel(const double& mu, const double& sigma);
  void laserQueueThread();
};
}  // namespace gazebo
