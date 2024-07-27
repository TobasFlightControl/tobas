#pragma once

#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
class GazeboWorldContactsPlugin : public ModelPlugin
{
  static constexpr char kPluginName[] = "world_contacts_plugin";

  using self = GazeboWorldContactsPlugin;
  using super = ModelPlugin;

public:
  explicit GazeboWorldContactsPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;

  physics::ContactManager* contact_manager_;
  event::ConnectionPtr update_connection_;

  ros::Publisher contacts_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);
};
}  // namespace gazebo
