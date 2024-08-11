#pragma once

#include <rclcpp/rclcpp.hpp>
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
    // pointer to ros node

  // SDF parameters
  std::string frame_name_;  // frame transform name, should match link name
  double noise_stddev_;     // Gaussian noise

  sensors::SensorPtr parent_sensor_;  // The parent sensor
  sensors::RaySensorPtr parent_ray_sensor_;
  physics::MultiRayShapePtr shape_;

  size_t laser_connect_count_ = 0;  // Keep track of number of connctions
  chrono::steady_clock::duration last_update_time_;
  chrono::steady_clock::duration sim_time_;

  boost::mutex lock_;               // A mutex to lock access to fields that are used in message callbacks
  rclcpp::CallbackQueue laser_queue_;  // Custom Callback Queue
  boost::thread callback_laser_queue_thread_;

  PublisherPtr<> pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void putLaserData(const chrono::steady_clock::duration& update_time);
  void laserConnect();
  void laserDisconnect();
  double gaussianKernel(const double& mu, const double& sigma);
  void laserQueueThread();
};
}  // namespace gazebo
