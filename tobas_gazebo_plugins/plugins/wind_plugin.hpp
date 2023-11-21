#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_tools/dryden_wind_model.hpp>
#include <tobas_gazebo_plugins/SetWindParameters.h>

#include "../include/tobas_gazebo_plugins/common.hpp"

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
  // Constants
  static constexpr char kPluginName[] = "wind_plugin";

  // Default parameters
  static constexpr double kDefaultMeanWindSpeed = 0.;          // [m/s]
  static constexpr double kDefaultConstantWindDirection = 0.;  // [rad]
  static constexpr double kDefaultGustSpeedFactor = 1.;        // [-]
  static constexpr double kDefaultGustDuration = 5.;           // [s]
  static constexpr double kDefaultGustInterval = 10.;          // [s]

  using self = GazeboWindPlugin;
  using super = ModelPlugin;

public:
  explicit GazeboWindPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  enum gust_state_t : uint8_t
  {
    GUST,
    NO_GUST,
  };

  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;

  // Wind parameters
  double mean_speed_ = kDefaultMeanWindSpeed;
  double direction_ = kDefaultConstantWindDirection;
  double gust_speed_factor_ = kDefaultGustSpeedFactor;
  double gust_duration_ = kDefaultGustDuration;
  double gust_interval_ = kDefaultGustInterval;

  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  common::Time prev_sim_time_;
  common::Time gust_state_change_time_;
  gust_state_t gust_state_ = NO_GUST;
  tobas::DrydenSimulator dryden_;

  ros::Publisher wind_pub_;

  ros::ServiceServer set_wind_params_srv_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);

  bool setWindParametersCb(
    tobas_gazebo_plugins::SetWindParametersRequest& req,
    tobas_gazebo_plugins::SetWindParametersResponse& res);
};
}  // namespace gazebo
