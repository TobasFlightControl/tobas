#pragma once

#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_gazebo_msgs/ContactStates.h>

namespace gazebo
{
class GazeboTetherStationForcePlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "tether_station_force_plugin";

  // Default parameters
  static constexpr double kDefaultInitTension = 1.;  // [N]
  static constexpr double kDefaultInitTension = 1.;  // [N]

  using self = GazeboTetherStationForcePlugin;
  using super = ModelPlugin;

public:
  explicit GazeboTetherStationForcePlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  ignition::math::Vector3d W_Pos_WP_;
  ignition::math::Vector3d B_Pos_BQ_;
  double init_tension_;     // [N]
  double init_max_length_;  // [m]

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  bool first_contact_detected_ = false;
  tobas_gazebo_msgs::ContactStatesConstPtr contacts_;

  ros::Subscriber contacts_sub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);

  bool isThis(const std::string& name);
  bool isPlane(const uint32_t& shape);
  bool isContactWithPlane();

  void contactStatesCb(const tobas_gazebo_msgs::ContactStatesConstPtr& contacts);
};
}  // namespace gazebo
