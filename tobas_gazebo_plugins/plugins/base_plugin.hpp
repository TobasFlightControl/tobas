#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class GazeboBasePlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "base_plugin";
  static constexpr char kRotorJointNames[] = "rotorJointNames";

  using super = ModelPlugin;

public:
  explicit GazeboBasePlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::vector<std::string> rotor_joint_names_;

  // Gazebo objects
  physics::ModelPtr model_;
  std::vector<physics::JointPtr> rotor_joints_;
  event::ConnectionPtr update_connection_;

  // PubSub
  ros::Publisher rotor_speeds_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);
  void registerPubSub();
};
}  // namespace gazebo
