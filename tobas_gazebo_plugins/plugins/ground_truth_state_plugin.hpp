#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_msgs/PoseTwist.h>

namespace gazebo
{
class GazeboGroundTruthStatePlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "ground_truth_state_plugin";
  static constexpr char kStatePubTopic[] = "ground_truth/pose_twist";

  using super = ModelPlugin;
  using StateMsg = tobas_msgs::PoseTwist;

public:
  explicit GazeboGroundTruthStatePlugin();

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;
  StateMsg state_msg_;

  ros::Publisher state_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo&);
};
}  // namespace gazebo
