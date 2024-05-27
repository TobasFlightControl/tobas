#pragma once

#include <map>
#include <functional>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <std_srvs/Empty.h>

#include <tobas_std_tools/first_order_filter.hpp>
#include <tobas_msgs/RotorState.h>

#include "../include/tobas_gazebo_plugins/common.hpp"

namespace gazebo
{
class GazeboBatteryPlugin : public ModelPlugin
{
  // Constants
  static constexpr char kPluginName[] = "battery_plugin";
  static constexpr double kSagCapRate = 0.2;  // [-] 放電特性が急激に変化する点における電気残率

  // Default parameters
  static constexpr double kDefaultVoltageNoiseStddev = 0.1;  // [V]
  static constexpr double kDefaultCurrentNoiseStddev = 0.;   // [A]

  using self = GazeboBatteryPlugin;
  using super = ModelPlugin;

public:
  explicit GazeboBatteryPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  ros::NodeHandle nh_;

  // SDF parameters
  std::string ns_;
  double max_voltage_;  // [V] 満充電時の電圧
  double sag_voltage_;  // [V] 放電特性が急激に変化する電圧．LiPoなら1セルあたり3.4Vくらい．
  double max_current_;  // [A] 最大電流
  double capacity_;     // [As] 電気容量
  double registance_;   // [Ω] 内部抵抗値
  double voltage_noise_stddev_;  // [V] 電圧の観測ノイズの標準偏差
  double current_noise_stddev_;  // [A] 電流の観測ノイズの標準偏差
  size_t num_rotors_;

  std::vector<double> currents_;  // [A] 各モータに流れる電流
  double q_;                      // [As] 現在の電気量
  common::Time t_last_;
  event::ConnectionPtr update_connection_;

  // Random generator
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution voltage_noise_;
  NormalDistribution current_noise_;

  // Publishers
  ros::Publisher battery_pub_;
  ros::Publisher battery_gt_pub_;

  // Subscribers
  std::vector<ros::Subscriber> rotor_state_subs_;
  std::vector<std::function<void(const tobas_msgs::RotorStateConstPtr&)>> rotor_state_cbs_;

  // Service servers
  ros::ServiceServer charge_srv_;

  void getSdfParams(sdf::ElementPtr sdf);
  void registerPubSub();
  void onUpdate(const common::UpdateInfo& info);
  double currentVoltage();

  bool chargeCb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);
};
}  // namespace gazebo
