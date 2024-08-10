#pragma once

#include <rclcpp/rclcpp.hpp>
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



public:
  explicit GazeboBasePlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  rclcpp::Node::SharedPtr node_;

  // SDF parameters
  std::vector<std::string> rotor_joint_names_;

  // Gazebo objects
  physics::ModelPtr model_;
  std::vector<physics::JointPtr> rotor_joints_;
  event::ConnectionPtr update_connection_;

  // PubSub
  PublisherPtr<> rotor_speeds_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void onUpdate(const common::UpdateInfo& info);
  void registerPubSub();
};
}  // namespace gazebo
