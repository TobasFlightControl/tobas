#pragma once

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class GazeboGroundTruthStatePlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "ground_truth_state_plugin";

  using self = GazeboGroundTruthStatePlugin;


public:
  explicit GazeboGroundTruthStatePlugin();

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  rclcpp::Node::SharedPtr node_;

  // SDF parameters
  std::string link_name_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  PublisherPtr<> odom_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void onUpdate(const common::UpdateInfo&);
};
}  // namespace gazebo
