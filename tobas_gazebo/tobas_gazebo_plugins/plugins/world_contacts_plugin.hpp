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

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  rclcpp::Node::SharedPtr node_;

  physics::ContactManager* contact_manager_;
  event::ConnectionPtr update_connection_;

  PublisherPtr<> contacts_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void onUpdate(const common::UpdateInfo& info);
};
}  // namespace gazebo
