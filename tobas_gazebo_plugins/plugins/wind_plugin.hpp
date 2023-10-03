#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "wind_plugin";
static constexpr double kMinimumAltitude = 1.;          // [m]
static constexpr double kLowAltitudeThreshold = 1000.;  // [ft]

// Default parameters
static constexpr double kDefaultMeanWindSpeed = 0.;          // [m/s]
static constexpr double kDefaultConstantWindDirection = 0.;  // [rad]
static constexpr double kDefaultGustSpeedFactor = 1.;        // [-]
static constexpr double kDefaultGustDuration = 5.;           // [s]
static constexpr double kDefaultGustInterval = 10.;          // [s]

/**
 * @brief Dryden Wind Turbulence Model (Low-Altitude Model)
 * https://jp.mathworks.com/help/aeroblks/drydenwindturbulencemodeldiscrete.html
 */
class GazeboWindPlugin : public ModelPlugin
{
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
  std::string wind_topic_;
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
  ignition::math::Vector3d turb_B_ = zero3;  // [m/s] Turbulence (Dryden model)

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution noise_;

  // PubSub
  ros::Publisher wind_pub_;

  void getSdfParams(sdf::ElementPtr sdf);
  void onUpdate(const common::UpdateInfo& info);
  void registerPubSub();
};
}  // namespace gazebo
