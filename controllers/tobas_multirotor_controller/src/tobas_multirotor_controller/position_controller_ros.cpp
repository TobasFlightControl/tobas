#include <eigen_conversions/eigen_msg.h>

#include <dh_ros_tools/rosparam.hpp>

#include <tobas_msgs/FrameId.h>

#include "../../include/tobas_multirotor_controller/position_controller_ros.hpp"
#include "../../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_multirotor_controller
{
PositionControllerRos::PositionControllerRos()
  : super(),
    is_initialized_(false),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &PositionControllerRos::checkTopicsTimerCb,
      this),
    server_(ros::NodeHandle(kCtrlName))
{
  getRosParams();

  pos_controller_.reset(new PositionController(dynamic_params_));
  vel_yaw_out_.frame_id.frame_id = tobas_msgs::FrameId::GLOBAL;

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f =
    boost::bind(&PositionControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void PositionControllerRos::getRosParams()
{
  dh_ros::getParam(kCtrlName + "/natural_frequency", dynamic_params_.natural_freq);
  dh_ros::getParam(kCtrlName + "/damping_ratio", dynamic_params_.damp_ratio);
}

void PositionControllerRos::registerPublishers()
{
  vel_yaw_pub_ = nh_.advertise<tobas_msgs::VelocityYaw>("command/velocity_yaw", 1);
}

void PositionControllerRos::registerSubscribers()
{
  base_state_sub_ = nh_.subscribe("base_state", 1, &PositionControllerRos::baseStateCb, this);
  pos_yaw_sub_ =
    nh_.subscribe("command/position_yaw", 1, &PositionControllerRos::targetPositionCb, this);
}

void PositionControllerRos::initialize(const tobas_msgs::BaseState& bs)
{
  // 最初は暴れるのを防ぐために現在の状態を目標状態にする
  pos_yaw_in_.pos = bs.pose.pos;
  pos_yaw_in_.pos(2) += kInitialTargetAltitude;  // Z座標に遊びを設ける
  pos_yaw_in_.yaw = bs.pose.euler.yaw;
}

void PositionControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_.natural_freq = cfg.natural_frequency;
  dynamic_params_.damp_ratio = cfg.damping_ratio;
}

void PositionControllerRos::baseStateCb(const tobas_msgs::BaseState& bs)
{
  if (!is_initialized_)
  {
    check_topics_timer_.stop();
    initialize(bs);
    is_initialized_ = true;
    rosInfo("Position controller is ready.");
    return;
  }

  // Compute target velocity and yaw angle
  pos_controller_->update(bs.pose.pos, pos_yaw_in_.pos, vel_yaw_out_.vel);
  vel_yaw_out_.yaw = pos_yaw_in_.yaw;  // ヨー角は位置指令をそのまま流す

  // Publish VelocityYaw message
  vel_yaw_pub_.publish(vel_yaw_out_);
}

void PositionControllerRos::targetPositionCb(const tobas_msgs::PositionYaw& pos_yaw)
{
  pos_yaw_in_ = pos_yaw;
}

void PositionControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  rosWarn("Base state is not received yet.");
}

void PositionControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  updateDynamicParams(cfg);

  pos_controller_->reconfigure(dynamic_params_);
}
}  // namespace tobas_multirotor_controller
