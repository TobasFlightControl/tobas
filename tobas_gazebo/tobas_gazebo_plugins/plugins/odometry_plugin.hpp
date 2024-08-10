#pragma once

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <opencv2/core/core.hpp>

#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/random.hpp"

namespace gazebo
{
class GazeboOdometryPlugin : public SensorPlugin
{  // Constants
  static constexpr char kPluginName[] = "odometry_plugin";

  // Default values
  static constexpr double kDefaultCovarianceImageScale = 1.;



public:
  explicit GazeboOdometryPlugin();

  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override;

private:
  rclcpp::Node::SharedPtr node_;

  // SDF parameters
  std::string link_name_;
  math::Vector3d offset_;  // B_Pos_BS
  math::Vector3d noise_normal_position_;
  math::Vector3d noise_normal_rotation_;
  math::Vector3d noise_normal_linvel_;
  math::Vector3d noise_normal_angvel_;
  math::Vector3d noise_uniform_position_;
  math::Vector3d noise_uniform_rotation_;
  math::Vector3d noise_uniform_linvel_;
  math::Vector3d noise_uniform_angvel_;
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

  PublisherPtr<> odometry_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void setRandomDistributions();
  void registerPublishers();
  void onUpdate();
  void addNoise(gz::math::Pose3d& pose, gz::math::Vector3d& linvel, gz::math::Vector3d& angvel) const;
  void publishOdomMsg(gz::math::Pose3d& pose, gz::math::Vector3d& linvel, gz::math::Vector3d& angvel)
    const;
};
}  // namespace gazebo
