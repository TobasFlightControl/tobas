#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <opencv2/core/core.hpp>
#include <nav_msgs/Odometry.h>

#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/random.hpp"

namespace gazebo
{
class GazeboOdometryPlugin : public SensorPlugin
{  // Constants
  static constexpr char kPluginName[] = "odometry_plugin";

  // Default values
  static constexpr double kDefaultCovarianceImageScale = 1.;

  using super = SensorPlugin;

public:
  explicit GazeboOdometryPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  SdfVector3 offset_;  // B_Pos_BS
  SdfVector3 noise_normal_position_;
  SdfVector3 noise_normal_rotation_;
  SdfVector3 noise_normal_linvel_;
  SdfVector3 noise_normal_angvel_;
  SdfVector3 noise_uniform_position_;
  SdfVector3 noise_uniform_rotation_;
  SdfVector3 noise_uniform_linvel_;
  SdfVector3 noise_uniform_angvel_;
  cv::Mat covariance_image_;
  double cov_image_scale_;

  // Noise distributions
  NormalDistribution3dPtr position_n_;
  NormalDistribution3dPtr rotation_n_;
  NormalDistribution3dPtr linvel_n_;
  NormalDistribution3dPtr angvel_n_;
  UniformDistribution3dPtr position_u_;
  UniformDistribution3dPtr rotation_u_;
  UniformDistribution3dPtr linvel_u_;
  UniformDistribution3dPtr angvel_u_;

  physics::WorldPtr world_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  std::random_device rnd_dev_;
  nav_msgs::Odometry odom_msg_;

  ros::Publisher odometry_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void fillMessageStaticParts();
  void setRandomDistributions();
  void registerPublishers();
  void onUpdate();
  void addNoise(ignition::math::Pose3d& pose, ignition::math::Vector3d& linvel, ignition::math::Vector3d& angvel);
};
}  // namespace gazebo
