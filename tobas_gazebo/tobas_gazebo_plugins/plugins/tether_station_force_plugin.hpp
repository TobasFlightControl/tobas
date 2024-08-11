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

  // Default parameters
  static constexpr double kDefaultInitTension = 1.;       // [N]
  static constexpr double kDefaultInitMaxLength = 5.;     // [N]
  static constexpr double kDefaultYoungModulus = 200.;    // [MPa] 低密度ポリエチレン
  static constexpr double kDefaultCrossSectionArea = 1.;  // [mm^2]

  using self = GazeboTetherStationForcePlugin;


public:
  explicit GazeboTetherStationForcePlugin();

protected:
  void Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&) override;

private:


  // SDF parameters
  std::string link_name_;
  gz::math::Vector3d W_Pos_WP_;
  gz::math::Vector3d B_Pos_BQ_;
  double init_tension_;     // [N]
  double init_max_length_;  // [m]
  double young_;            // [MPa] ヤング率 (Young Modulus)
  double csa_;              // [mm^2] 断面積 (Cross-Sectional Area)

  physics::ModelPtr model_;
  physics::LinkPtr link_;


  bool first_contact_detected_ = false;
  tobas_gazebo_msgs::ContactStates::ConstSharedPtr contacts_;
  tobas_gazebo_msgs::TetherParams params_;

  SubscriberPtr<> contacts_sub_;

  ServicePtr<> get_params_ss_;
  ServicePtr<> set_params_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);


  bool isThis(const std::string& name);
  bool isPlane(const uint32_t& shape);
  bool isContactWithPlane();

  void contactStatesCb(const tobas_gazebo_msgs::ContactStates::ConstSharedPtr& contacts);
  bool getParamsCb(tobas_gazebo_msgs::GetTetherParamsRequest& req, tobas_gazebo_msgs::GetTetherParamsResponse& res);
  bool setParamsCb(tobas_gazebo_msgs::SetTetherParamsRequest& req, tobas_gazebo_msgs::SetTetherParamsResponse& res);
};
}  // namespace gazebo
