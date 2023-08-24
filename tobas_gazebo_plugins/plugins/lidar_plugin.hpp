#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/PubQueue.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

namespace gazebo
{
// Constants
static const std::string kPluginName = "lidar_plugin";

// Default values
static const std::string kDefaultFrameName = "world";
static const std::string kDefaultTopicName = "point_cloud";

class GazeboLidarPlugin : public RayPlugin
{
  using super = RayPlugin;

public:
  explicit GazeboLidarPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle ros_node_;
  gazebo::transport::NodePtr gazebo_node_;

  // SDF parameters
  std::string frame_name_;  // Frame transform name, should match link name
  std::string topic_name_;  // Topic name

  laser_geometry::LaserProjection laser_projector_;
  sensor_msgs::LaserScan laser_msg_;
  sensor_msgs::PointCloud pc_msg_;

  uint32_t laser_connect_count_ = 0;  // Keep track of number of connctions
  std::string robot_namespace_;       // For setting ROS name space
  physics::WorldPtr world_;
  sensors::RaySensorPtr parent_;  // The parent sensor
  boost::thread deferred_load_thread_;

  ros::Publisher pc_pub_;
  gazebo::transport::SubscriberPtr laser_sub_;
  PubQueue<sensor_msgs::PointCloud>::Ptr pc_pub_queue_;
  PubMultiQueue pmq_;  // Prevents blocking

  void loadThread();
  void getSdfParams(sdf::ElementPtr sdf);

  void laserConnect();
  void laserDisconnect();

  /* Convert new Gazebo message to ROS message and publish it. */
  void onScan(ConstLaserScanStampedPtr& msg);
};
}  // namespace gazebo
