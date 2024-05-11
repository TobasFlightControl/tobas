#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/highgui/highgui.hpp>

#include <tobas_std_tools/math.hpp>
#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_tools/constants.hpp>

#include "./odometry_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_std;

namespace gazebo
{
GazeboOdometryPlugin::GazeboOdometryPlugin() : super()
{
}

void GazeboOdometryPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);
  fillMessageStaticParts();
  setRandomDistributions();

  world_ = physics::get_world(sensor->WorldName());
  link_ = dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name_));
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  registerPublishers();
  update_connection_ = sensor->ConnectUpdated(boost::bind(&GazeboOdometryPlugin::onUpdate, this));
}

void GazeboOdometryPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "offset", offset_, zero3);
  getSdfParam(sdf, "noiseNormalPosition", noise_normal_position_, zero3);
  getSdfParam(sdf, "noiseNormalRotation", noise_normal_rotation_, zero3);
  getSdfParam(sdf, "noiseNormalLinearVelocity", noise_normal_linvel_, zero3);
  getSdfParam(sdf, "noiseNormalAngularVelocity", noise_normal_angvel_, zero3);
  getSdfParam(sdf, "noiseUniformPosition", noise_uniform_position_, zero3);
  getSdfParam(sdf, "noiseUniformRotation", noise_uniform_rotation_, zero3);
  getSdfParam(sdf, "noiseUniformLinearVelocity", noise_uniform_linvel_, zero3);
  getSdfParam(sdf, "noiseUniformAngularVelocity", noise_uniform_angvel_, zero3);

  if (sdf->HasElement("covarianceImage"))
  {
    const auto image_name = sdf->GetElement("covarianceImage")->Get<string>();
    covariance_image_ = cv::imread(image_name, cv::IMREAD_GRAYSCALE);
    if (covariance_image_.data == nullptr)
      gzthrow(kPluginName << ": Loading covariance image " << image_name << " failed.");

    getSdfParam(
      sdf, "covarianceImageScale", cov_image_scale_, kDefaultCovarianceImageScale, POSITIVE);
  }
}

void GazeboOdometryPlugin::fillMessageStaticParts()
{
  // Fill in frame ids
  odom_msg_.header.frame_id = "world";
  odom_msg_.child_frame_id = link_name_;

  // Fill in pose covariance
  Map<Matrix6d> pose_covariance(odom_msg_.pose.covariance.data());
  Vector6d pose_covd;
  pose_covd << sqr(noise_normal_position_.X()), sqr(noise_normal_position_.Y()),
    sqr(noise_normal_position_.Z()), sqr(noise_normal_rotation_.X()),
    sqr(noise_normal_rotation_.Y()), sqr(noise_normal_rotation_.Z());
  pose_covariance = pose_covd.asDiagonal();

  // Fill in twist covariance
  Map<Matrix6d> twist_covariance(odom_msg_.twist.covariance.data());
  Vector6d twist_covd;
  twist_covd << sqr(noise_normal_linvel_.X()), sqr(noise_normal_linvel_.Y()),
    sqr(noise_normal_linvel_.Z()), sqr(noise_normal_angvel_.X()), sqr(noise_normal_angvel_.Y()),
    sqr(noise_normal_angvel_.Z());
  twist_covariance = twist_covd.asDiagonal();
}

void GazeboOdometryPlugin::setRandomDistributions()
{
  position_n_.reset(new NormalDistribution3d(rnd_dev_, zero3, noise_normal_position_));
  rotation_n_.reset(new NormalDistribution3d(rnd_dev_, zero3, noise_normal_rotation_));
  linvel_n_.reset(new NormalDistribution3d(rnd_dev_, zero3, noise_normal_linvel_));
  angvel_n_.reset(new NormalDistribution3d(rnd_dev_, zero3, noise_normal_angvel_));
  position_u_.reset(
    new UniformDistribution3d(rnd_dev_, -noise_uniform_position_, noise_uniform_position_));
  rotation_u_.reset(
    new UniformDistribution3d(rnd_dev_, -noise_uniform_rotation_, noise_uniform_rotation_));
  linvel_u_.reset(
    new UniformDistribution3d(rnd_dev_, -noise_uniform_linvel_, noise_uniform_linvel_));
  angvel_u_.reset(
    new UniformDistribution3d(rnd_dev_, -noise_uniform_angvel_, noise_uniform_angvel_));
}

void GazeboOdometryPlugin::registerPublishers()
{
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("/" + ns_ + "/" + tobas::kExternalOdomTopic, 1);
}

void GazeboOdometryPlugin::onUpdate()
{
  // ベースフレームの状態を取得
  const auto& T_W_B = link_->WorldPose();
  const auto B_Linvel_WB = link_->RelativeLinearVel();
  const auto B_Angvel_WB = link_->RelativeAngularVel();

  // センサフレームに変換
  const ignition::math::Pose3d T_B_S(offset_, ignition::math::Quaterniond::Identity);
  auto T_W_S = T_W_B * T_B_S;
  auto B_Linvel_WS = B_Linvel_WB + B_Angvel_WB.Cross(offset_);
  auto B_Angvel_WS = B_Angvel_WB;  // オフセットが並進のみならば角速度は同じ

  // Check if odometry is available
  if (covariance_image_.data != nullptr)
  {
    // Image is always centered around the origin
    const auto width = covariance_image_.cols;
    const auto height = covariance_image_.rows;
    const auto x = static_cast<int>(floor(T_W_B.Pos().X() / cov_image_scale_)) + width / 2;
    const auto y = static_cast<int>(floor(T_W_B.Pos().Y() / cov_image_scale_)) + height / 2;

    if (0 <= x && x < width && 0 <= y && y < height)
    {
      const auto pixel_value = covariance_image_.at<uint8_t>(y, x);
      if (pixel_value == 0)
        return;  // Unable to get odometry
    }
  }

  // Add noise to the true values
  addNoise(T_W_S, B_Linvel_WS, B_Angvel_WS);

  // Fill in odometry message
  timeGazeboToRos(world_->SimTime(), odom_msg_.header.stamp);
  poseGazeboToRos(T_W_B, odom_msg_.pose.pose);
  vectorGazeboToRos(B_Linvel_WS, odom_msg_.twist.twist.linear);
  vectorGazeboToRos(B_Angvel_WS, odom_msg_.twist.twist.angular);

  odometry_pub_.publish(odom_msg_);
}

void GazeboOdometryPlugin::addNoise(
  ignition::math::Pose3d& pose,
  ignition::math::Vector3d& linvel,
  ignition::math::Vector3d& angvel)
{
  // Add position noise
  pose.Pos() += position_n_->get() + position_u_->get();

  // Add rotation noise
  const auto theta = rotation_n_->get() + rotation_u_->get();
  const auto q_n = angleAxisToQuaternion(theta);
  pose.Rot() *= q_n;
  pose.Rot().Normalize();

  // Add linear velocity noise
  linvel += linvel_n_->get() + linvel_u_->get();

  // Add angular velocity noise
  angvel += angvel_n_->get() + angvel_u_->get();
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboOdometryPlugin);
}  // namespace gazebo
