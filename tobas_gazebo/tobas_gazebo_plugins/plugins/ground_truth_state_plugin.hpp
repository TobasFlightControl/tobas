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

  using self = GazeboGroundTruthStatePlugin;


public:
  explicit GazeboGroundTruthStatePlugin();

  void Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&) override;

private:


  // SDF parameters
  std::string link_name_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;


  PublisherPtr<> odom_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void onUpdate(const common::UpdateInfo&);
};
}  // namespace gazebo
