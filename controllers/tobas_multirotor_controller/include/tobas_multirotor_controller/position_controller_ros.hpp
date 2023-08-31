#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_multirotor_controller/ControllerConfig.h>

#include "./position_controller.hpp"

namespace tobas_multirotor_controller
{
class PositionControllerRos : public tobas::BaseNode
{
  static constexpr double kMaxCommandPositionDeviation = 100.;  // TODO

  using super = tobas::BaseNode;

  using ConfigType = tobas_multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit PositionControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
  bool is_initialized_;
  bool bs_received_;
  bool cmd_received_;
  tobas_msgs::BaseState bs_;
  tobas_msgs::PositionYaw pos_yaw_in_;   // 受け取る位置コマンド
  tobas_msgs::VelocityYaw vel_yaw_out_;  // 発行する速度コマンド

  PositionController pos_controller_;

  // rosparams
  PositionControllerDynamicParams dynamic_params_;

  // PubSub
  ros::Publisher vel_yaw_pub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber pos_yaw_sub_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize();
  void updateDynamicParams(const ConfigType& cfg);

  void eventCb(const tobas_msgs::Event& event) override;
  void baseStateCb(const tobas_msgs::BaseState& bs);
  void targetPositionCb(const tobas_msgs::PositionYaw& pos_yaw);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_multirotor_controller
