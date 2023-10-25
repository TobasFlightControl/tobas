#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_tools/dryden_wind_model.hpp>

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
  double mean_speed_;         // [m/s] 地面からの高度20ftで測った平均風速
  double direction_;          // [rad] 風向 (ヨー角)
  double gust_speed_factor_;  // 定常風速に対する突風成分の風速の比率
  double gust_duration_;      // 突風の発生時間
  double gust_interval_;      // 突風が過ぎ去ってから次の突風が来るまでの時間

  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  common::Time prev_sim_time_ = 0;
  common::Time gust_state_change_time_ = 0;
  gust_state_t gust_state_ = NO_GUST;
  double gust_speed_ = 0.;
  tobas::DrydenSimulator dryden_;

  // PubSub
  ros::Publisher wind_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);
  void registerPubSub();
};
}  // namespace gazebo
