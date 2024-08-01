#include <tobas_tools/constants.hpp>
#include <tobas_msgs/RotorSpeeds.h>

#include "./base_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboBasePlugin::GazeboBasePlugin() : super()
{
}

void GazeboBasePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  // Get SDF parameters
  getSdfParams(sdf);

  // Store the pointer to the model
  model_ = model;

  // Get the pointer to the rotor joints
  for (const auto& joint_name : rotor_joint_names_)
  {
    const auto joint = model_->GetJoint(joint_name);
    if (joint == nullptr)
      gzthrow(kPluginName << ": Couldn't find specified joint \"" << joint_name << "\".");
    rotor_joints_.push_back(joint);
  }

  registerPubSub();

  // Listen to the update event
  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboBasePlugin::onUpdate, this, _1));
}

void GazeboBasePlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "rotorJointNames", rotor_joint_names_);
}

void GazeboBasePlugin::onUpdate(const common::UpdateInfo& info)
{
  // Publish rotor speeds
  const auto rotor_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
  timeGazeboToRos(info.simTime, rotor_speeds->header.stamp);
  for (const auto& joint : rotor_joints_)
  {
    const auto rot_speed_sim = joint->GetVelocity(0);
    const auto rot_speed_real = rot_speed_sim * kRotorSpeedSlowdownSim;
    rotor_speeds->speeds.push_back(abs(rot_speed_real));
  }
  rotor_speeds_pub_.publish(rotor_speeds);
}

void GazeboBasePlugin::registerPubSub()
{
  rotor_speeds_pub_ = node_.advertise<tobas_msgs::RotorSpeeds>("/" + ns_ + "/" + tobas::kRotorSpeedsTopic, 1);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBasePlugin);
}  // namespace gazebo
