#include <dh_std_tools/math.hpp>

#include "../../include/plugins/ground_truth_state_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"
#include "../../include/tobas_gazebo_plugins/conversions.hpp"

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
  if (!getSdfParam<string>(sdf, "robotNamespace", ns_))
  {
    gzthrow(kPluginName << ": Please specify a robotNamespace.");
  }

  if (!getSdfParam<string>(sdf, "linkName", link_name_))
  {
    gzthrow(kPluginName << ": Please specify a linkName.");
  }

  getSdfParam<string>(sdf, "stateTopic", state_topic_, kDefaultStateTopic);
}

void GazeboGroundTruthStatePlugin::onUpdate(const common::UpdateInfo&)
{
  common::Time cur_time = world_->SimTime();
  Pose3d T_W_B = link_->WorldPose();

  // Fill state message.
  timeGazeboToRos(cur_time, state_msg_.header.stamp);

  vectorGazeboToRos(T_W_B.Pos(), state_msg_.pose_vel.pose.position);

  const Quaterniond& q = T_W_B.Rot();
  tobas_msgs::Euler& e = state_msg_.pose_vel.pose.orientation;
  dh_std::quaternionToEuler(q.X(), q.Y(), q.Z(), q.W(), e.roll, e.pitch, e.yaw);

  vectorGazeboToRos(link_->WorldLinearVel(), state_msg_.pose_vel.twist.linear);
  vectorGazeboToRos(link_->RelativeAngularVel(), state_msg_.pose_vel.twist.angular);

  // Publish state message
  state_pub_.publish(state_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboGroundTruthStatePlugin);
}  // namespace gazebo
