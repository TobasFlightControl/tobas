#pragma once

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <dh_ros_tools/timer.hpp>

#include <tobas_msgs/PoseVelStamped.h>
#include <tobas_msgs/PositionYaw.h>
#include <tobas_msgs/VelocityYaw.h>
#include <tobas_multirotor_controller/ControllerConfig.h>

#include "./position_controller.hpp"

class PositionControllerRos
{
  using ConfigType = tobas_multirotor_controller::ControllerConfig;
  using ConfigServer = dynamic_reconfigure::Server<ConfigType>;

public:
  explicit PositionControllerRos();

private:
  ros::NodeHandle nh_;

  // rosparams
  std::string drone_name_;
  PositionControllerDynamicParams dynamic_params_;

  bool is_initialized_;
  Eigen::Vector3d cur_pos_;
  Eigen::Vector3d target_pos_;
  double target_yaw_;
  Eigen::Vector3d target_vel_;
  tobas_msgs::VelocityYaw vel_yaw_;

  std::shared_ptr<PositionController> pos_controller_;

  // PubSub
  ros::Publisher vel_yaw_pub_;
  ros::Subscriber base_state_sub_;
  ros::Subscriber pos_yaw_sub_;

  ConfigServer server_;               // Dynamic Reconfigure
  dh_ros::Timer check_topics_timer_;  // Check if messages are received or not.

  void getRosParams();
  void registerPubSub();
  void initialize(const tobas_msgs::PoseVelStamped& bs);
  void updateDynamicParams(const ConfigType& cfg);

  void baseStateCb(const tobas_msgs::PoseVelStamped& bs);
  void targetPositionCb(const tobas_msgs::PositionYaw& pos_yaw);
  void dynamicReconfigureCb(const ConfigType& cfg, uint32_t level);
  void checkTopicsTimerCb(const ros::TimerEvent&);
};
