#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <dh_ros_tools/node.hpp>

#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_multirotor_controller/ControllerConfig.h>

#include "./position_controller.hpp"

namespace tobas_multirotor_controller
{
class PositionControllerRos : public dh_ros::BaseNode
{
  using super = dh_ros::BaseNode;

  using ConfigType = tobas_multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit PositionControllerRos();

private:
  bool is_initialized_;
  tobas_msgs::PositionYaw pos_yaw_in_;   // 受け取る位置コマンド
  tobas_msgs::VelocityYaw vel_yaw_out_;  // 発行する速度コマンド

  std::shared_ptr<PositionController> pos_controller_;

  // rosparams
  PositionControllerDynamicParams dynamic_params_;

  // PubSub
  ros::Publisher vel_yaw_pub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber pos_yaw_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;
  void createTimers() override;

  void initialize(const tobas_msgs::BaseState& bs);
  void updateDynamicParams(const ConfigType& cfg);

  void baseStateCb(const tobas_msgs::BaseState& bs);
  void targetPositionCb(const tobas_msgs::PositionYaw& pos_yaw);

  void checkTopicsTimerCb(const ros::TimerEvent& event) override;
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
};
}  // namespace tobas_multirotor_controller
