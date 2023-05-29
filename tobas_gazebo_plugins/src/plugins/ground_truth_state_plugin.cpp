#include <dh_std_tools/geometry.hpp>

#include "../../include/plugins/ground_truth_state_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboGroundTruthStatePlugin::GazeboGroundTruthStatePlugin() : super()
{
}

void GazeboGroundTruthStatePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get SDF parameters
  getSdfParams(sdf);

  // Store the pointer to the model
  model_ = model;
  world_ = model_->GetWorld();

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  // Fill the static parts of the state message
  state_msg_.header.frame_id = link_name_;

  // Advertise publisher
  state_pub_ = nh_.advertise<StateMsg>("/" + ns_ + "/" + state_topic_, 1);

  // Listen to the update event
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&GazeboGroundTruthStatePlugin::onUpdate, this, _1));
}

void GazeboGroundTruthStatePlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "stateTopic", state_topic_, kDefaultStateTopic);
}

void GazeboGroundTruthStatePlugin::onUpdate(const common::UpdateInfo&)
{
  const auto T_W_B = link_->WorldPose();

  // Update time stamp
  timeGazeboToRos(world_->SimTime(), state_msg_.header.stamp);

  // Update position
  vectorGazeboToKDL(T_W_B.Pos(), state_msg_.pose.pos);

  // Update rotation
  const auto& q = T_W_B.Rot();
  auto& e = state_msg_.pose.euler;
  dh_std::quaternionToEuler(q.X(), q.Y(), q.Z(), q.W(), e.roll, e.pitch, e.yaw);

  // Update linear velocity (Local)
  vectorGazeboToKDL(link_->RelativeLinearVel(), state_msg_.twist.vel);

  // Update angular velocity (Local)
  vectorGazeboToKDL(link_->RelativeAngularVel(), state_msg_.twist.rot);

  // Publish state message
  state_pub_.publish(state_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGroundTruthStatePlugin);
}  // namespace gazebo
