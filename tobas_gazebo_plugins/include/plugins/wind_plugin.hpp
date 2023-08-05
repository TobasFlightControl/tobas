#pragma once

#include <random>
#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <tobas_msgs/Wind.h>

#include "../../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
// Constants
static const std::string kPluginName = "wind_plugin";
static constexpr double kMinimumAltitude = 0.1;
static constexpr double kLowAltitudeThreshold = 1000.;  // [ft]

// Default values
static constexpr double kDefaultMeanWindSpeed = 0.;
static constexpr double kDefaultConstantWindDirection = 0.;

class GazeboWindPlugin : public ModelPlugin
{
  using super = ModelPlugin;

public:
  explicit GazeboWindPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  std::string link_name_;
  std::string wind_topic_;
  double mean_speed_;  // [m/s] 地面からの高度20ftで測った平均風速
  double direction_;   // [rad] 風向 (ヨー角)

  physics::LinkPtr link_;
  event::ConnectionPtr update_connection_;

  tobas_msgs::Wind wind_;
  double prev_sim_time_;
  ignition::math::Vector3d const_wind_W_;  // 定常風 (World)
  double gust_u_, gust_v_, gust_w_;        // 突風成分 (World)

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
