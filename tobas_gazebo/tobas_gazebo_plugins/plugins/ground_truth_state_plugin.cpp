#include <tobas_std_tools/geometry.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/Odometry.hpp>

#include "./ground_truth_state_plugin.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboGroundTruthStatePlugin::GazeboGroundTruthStatePlugin() : super()
{
}

void GazeboGroundTruthStatePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  // Get SDF parameters
  getSdfParams(sdf);

  // Store the pointer to the model
  model_ = model;
  world_ = model_->GetWorld();

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  // Advertise publisher
  odom_pub_ = createPublisher<tobas_msgs::Odometry>("/" + ns_ + "/" + kOdometryGtTopic);

  // Listen to the update event
  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&self::onUpdate, this, _1));
}

void GazeboGroundTruthStatePlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
}

void GazeboGroundTruthStatePlugin::onUpdate(const common::UpdateInfo&)
{
  const auto& T_W_B = link_->WorldPose();

  // Create Pose & Twist message
  const auto odom =std::make_unique<tobas_msgs::Odometry>();
  odom->header.frame_id = link_name_;

  // Update time stamp
  timeGazeboToRos(world_->SimTime(), odom->header.stamp);

  // Update status
  odom->status = tobas_msgs::msg::Odometry::NO_ERROR;

  // Update position
  vectorGazeboToKDL(T_W_B.Pos(), odom->frame.p);

  // Update rotation
  const auto& q = T_W_B.Rot();
  odom->frame.M = kdl::Rotation::Quaternion(q.X(), q.Y(), q.Z(), q.W());

  // Update linear velocity (Local)
  vectorGazeboToKDL(link_->RelativeLinearVel(), odom->twist.vel);

  // Update angular velocity (Local)
  vectorGazeboToKDL(link_->RelativeAngularVel(), odom->twist.rot);

  // Update linear acceleration (Local)
  vectorGazeboToKDL(link_->RelativeLinearAccel(), odom->accel.linear);

  // Update angular acceleration (Local)
  vectorGazeboToKDL(link_->RelativeAngularAccel(), odom->accel.angular);

  // Publish state message
  odom_pub_->publish(odom);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGroundTruthStatePlugin);
}  // namespace gazebo
