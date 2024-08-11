#pragma once

#include <random>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_wind_model/dryden.hpp>
#include <tobas_gazebo_msgs/GetWindParams.h>
#include <tobas_gazebo_msgs/SetWindParams.h>

#include "../include/tobas_gazebo_plugins/common/common.hpp"

namespace gazebo
{
/**
 * @brief Modeling of Wind Phenomena and Analysis of Their Effects on UAV Trajectory Tracking
 * Performance [Siqueira+, 2017] の4つの風を実装． \n
 *
 * - Constant wind: \n
 * - Turbulance: https://jp.mathworks.com/help/aeroblks/drydenwindturbulencemodeldiscrete.html \n
 * - Wind gust: 1-cosine model (https://aero.w3.kanazawa-u.ac.jp/cgi-bin/wiki.cgi?page=DISTB) \n
 * - Wind shear: // TODO: An overview of various kinds of wind effects on unmanned aerial vehicle \n
 */
class GazeboWindPlugin : public ModelPlugin
{

  // Default parameters
  static constexpr double kDefaultMeanWindSpeed = 0.;          // [m/s]
  static constexpr double kDefaultConstantWindDirection = 0.;  // [rad]
  static constexpr double kDefaultGustSpeedFactor = 1.;        // [-]
  static constexpr double kDefaultGustDuration = 5.;           // [s]
  static constexpr double kDefaultGustInterval = 10.;          // [s]

  using self = GazeboWindPlugin;


public:
  explicit GazeboWindPlugin();

protected:
  void Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&) override;

private:
  enum gust_state_t : uint8_t
  {
    GUST,
    NO_GUST,
  };



  // SDF parameters
  std::string link_name_;

  physics::LinkPtr link_;


  tobas_gazebo_msgs::WindParams params_;
  common::Time prev_sim_time_;
  common::Time gust_state_change_time_;
  gust_state_t gust_state_ = NO_GUST;
  double gust_speed_ = 0.;
  tobas::DrydenSimulator dryden_;

  PublisherPtr<tobas_msgs::Wind> wind_pub_;

  ServicePtr<> get_params_ss_;
  ServicePtr<> set_params_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void onUpdate(const common::UpdateInfo& info);

  bool getParamsCb(tobas_gazebo_msgs::GetWindParamsRequest& req, tobas_gazebo_msgs::GetWindParamsResponse& res);
  bool setParamsCb(tobas_gazebo_msgs::SetWindParamsRequest& req, tobas_gazebo_msgs::SetWindParamsResponse& res);
};
}  // namespace gazebo
