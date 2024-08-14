#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class GazeboWorldContactsPlugin : public ModelPlugin
{

  using self = GazeboWorldContactsPlugin;


public:
  explicit GazeboWorldContactsPlugin();


  void Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&) override;

private:


  physics::ContactManager* contact_manager_;


  PublisherPtr<> contacts_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

};
}  // namespace gazebo
