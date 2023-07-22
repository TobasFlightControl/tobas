#pragma once

#include <dynamic_reconfigure/server.h>

#include <dh_kdl/euler.hpp>
#include <dh_ros_tools/timer.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/Event.h>
#include <tobas_msgs/BaseState.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_msgs/RollPitchYawThrust.h>
#include <tobas_multirotor_controller/ControllerConfig.h>

#include "./velocity_controller.hpp"
#include "./acceleration_controller.hpp"

namespace tobas_multirotor_controller
{
/**
 * @brief 速度制御器(PD) + 加速度制御器(解析計算)．
 * vx, vy, vz, yaw_rateで指令する．
 */
class VelocityControllerRos : public tobas::BaseNode
{
  using super = tobas::BaseNode;

  using ConfigType = tobas_multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit VelocityControllerRos();

private:
  uint8_t cmd_level_;
  tobas_msgs::BaseState cur_bs_;               // 現在のベースの状態
  tobas_msgs::RollPitchYawThrust rpy_thrust_;  // 姿勢+推力 (出力)
  KDL::Vector tar_vel_W_;
  KDL::Vector tar_acc_W_;

  bool is_initialized_;
  bool bs_received_;
  bool vel_yaw_received_;

  VelocityController vel_controller_;
  AccelerationController acc_controller_;

  // RosParams
  VelocityControllerDynamicParams dynamic_params_;

  // Timer
  dh_ros::Timer check_topics_timer_;

  // PubSub
  ros::Publisher rpy_thrust_pub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber vel_yaw_sub_;

  // Dynamic Reconfigure
  ConfigServer server_;

  void getRosParams() override;
  void registerPublishers() override;
  void registerSubscribers() override;

  bool isReady();
  void initialize();
  void updateDynamicParams(const ConfigType& cfg);
  void runOnce();

  void eventCb(const tobas_msgs::Event& event) override;
  void baseStateCb(const tobas_msgs::BaseState& bs);
  void velocityYawCb(const tobas_msgs::VelocityYaw& vel_yaw);

  void checkTopicsTimerCb(const ros::TimerEvent&);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t);
};
}  // namespace tobas_multirotor_controller
