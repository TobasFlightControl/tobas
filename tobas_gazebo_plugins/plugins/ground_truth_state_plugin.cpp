#include <dh_std_tools/geometry.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/PoseTwist.h>

#include "./ground_truth_state_plugin.hpp"
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
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  // Advertise publisher
  pt_pub_ = nh_.advertise<tobas_msgs::PoseTwist>("/" + ns_ + "/" + tobas::kPoseTwistGtTopic, 1);

  // Listen to the update event
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
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
  auto pt = boost::make_shared<tobas_msgs::PoseTwist>();
  pt->header.frame_id = link_name_;

  // Update time stamp
  timeGazeboToRos(world_->SimTime(), pt->header.stamp);

  // Update position
  vectorGazeboToKDL(T_W_B.Pos(), pt->pose.pos);

  // Update rotation
  const auto& q = T_W_B.Rot();
  auto& e = pt->pose.euler;
  dh_std::quaternionToEuler(q.X(), q.Y(), q.Z(), q.W(), e.roll, e.pitch, e.yaw);

  // Update linear velocity (Local)
  vectorGazeboToKDL(link_->RelativeLinearVel(), pt->twist.vel);

  // Update angular velocity (Local)
  vectorGazeboToKDL(link_->RelativeAngularVel(), pt->twist.rot);

  // Update linear acceleration (Local)
  vectorGazeboToKDL(link_->RelativeLinearAccel(), pt->accel.linear);

  // Update angular acceleration (Local)
  vectorGazeboToKDL(link_->RelativeAngularAccel(), pt->accel.angular);

  // Publish state message
  pt_pub_.publish(pt);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGroundTruthStatePlugin);
}  // namespace gazebo
