#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_gazebo_msgs/ContactStates.h>
#include <tobas_gazebo_msgs/GetTetherParams.h>
#include <tobas_gazebo_msgs/SetTetherParams.h>

namespace gazebo
{
class GazeboTetherStationForcePlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "tether_station_force_plugin";

  // Default parameters
  static constexpr double kDefaultInitTension = 1.;       // [N]
  static constexpr double kDefaultInitMaxLength = 5.;     // [N]
  static constexpr double kDefaultYoungModulus = 200.;    // [MPa] 低密度ポリエチレン
  static constexpr double kDefaultCrossSectionArea = 1.;  // [mm^2]

  using self = GazeboTetherStationForcePlugin;
  using super = ModelPlugin;

public:
  explicit GazeboTetherStationForcePlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  rclcpp::NodeHandle node_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  ignition::math::Vector3d W_Pos_WP_;
  ignition::math::Vector3d B_Pos_BQ_;
  double init_tension_;     // [N]
  double init_max_length_;  // [m]
  double young_;            // [MPa] ヤング率 (Young Modulus)
  double csa_;              // [mm^2] 断面積 (Cross-Sectional Area)

  physics::ModelPtr model_;
  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  bool first_contact_detected_ = false;
  tobas_gazebo_msgs::ContactStatesConstPtr contacts_;
  tobas_gazebo_msgs::TetherParams params_;

  rclcpp::Subscriber contacts_sub_;

  rclcpp::ServiceServer get_params_ss_;
  rclcpp::ServiceServer set_params_ss_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);

  bool isThis(const std::string& name);
  bool isPlane(const uint32_t& shape);
  bool isContactWithPlane();

  void contactStatesCb(const tobas_gazebo_msgs::ContactStatesConstPtr& contacts);
  bool getParamsCb(tobas_gazebo_msgs::GetTetherParamsRequest& req, tobas_gazebo_msgs::GetTetherParamsResponse& res);
  bool setParamsCb(tobas_gazebo_msgs::SetTetherParamsRequest& req, tobas_gazebo_msgs::SetTetherParamsResponse& res);
};
}  // namespace gazebo
