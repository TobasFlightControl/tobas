#include <eigen_conversions/eigen_msg.h>

#include <dh_ros_tools/rosparam.hpp>

#include <tobas_msgs/FrameId.h>

#include "../../include/tobas_multirotor_controller/position_controller_ros.hpp"
#include "../../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_multirotor_controller
{
PositionControllerRos::PositionControllerRos() : is_initialized_(false)
{
  getRosParams();

  pos_controller_.reset(new PositionController(dynamic_params_));
  vel_yaw_.frame_id.frame_id = tobas_msgs::FrameId::GLOBAL;

  registerPubSub();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f =
    boost::bind(&PositionControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);

  // Start timer
  check_topics_timer_ = nh_.createTimer(
    ros::Duration(checkTopicsTimerPeriod), &PositionControllerRos::checkTopicsTimerCb, this);
}

void PositionControllerRos::getRosParams()
{
  dh_ros::getParam("/drone_name", drone_name_);

  dh_ros::getParam(ctrlPrefix + "/natural_frequency", dynamic_params_.natural_freq);
  dh_ros::getParam(ctrlPrefix + "/damping_ratio", dynamic_params_.damp_ratio);
}

void PositionControllerRos::registerPubSub()
{
  string drone_prefix = "/" + drone_name_;

  vel_yaw_pub_ =
    nh_.advertise<tobas_msgs::VelocityYaw>(drone_prefix + "/command/velocity_yaw", 1, false);

  base_state_sub_ =
    nh_.subscribe(drone_prefix + "/base_state", 1, &PositionControllerRos::baseStateCb, this);
  pos_yaw_sub_ = nh_.subscribe(
    drone_prefix + "/command/position_yaw", 1, &PositionControllerRos::targetPositionCb, this);
}

void PositionControllerRos::initialize(const tobas_msgs::PoseVelStamped& bs)
{
  // 最初は暴れるのを防ぐために現在の状態を目標状態にする
  tf::vectorMsgToEigen(bs.pose_vel.pose.position, target_pos_);
  target_pos_.z() += initialElevation;  // 地面との衝突を避けるためにZ座標だけは少し上げておく
  target_yaw_ = bs.pose_vel.pose.orientation.yaw;
}

void PositionControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_.natural_freq = cfg.natural_frequency;
  dynamic_params_.damp_ratio = cfg.damping_ratio;
}

void PositionControllerRos::baseStateCb(const tobas_msgs::PoseVelStamped& bs)
{
  if (!is_initialized_)
  {
    check_topics_timer_.stop();
    initialize(bs);
    is_initialized_ = true;
    dh_ros::rosInfo("Position controller is ready.");
    return;
  }

  tf::vectorMsgToEigen(bs.pose_vel.pose.position, cur_pos_);

  // Compute target velocity
  pos_controller_->update(cur_pos_, target_pos_, target_vel_);

  // Fill VelocityYaw messege
  tf::vectorEigenToMsg(target_vel_, vel_yaw_.velocity);
  vel_yaw_.yaw = target_yaw_;

  // Publish VelocityYaw message
  vel_yaw_pub_.publish(vel_yaw_);
}

void PositionControllerRos::targetPositionCb(const tobas_msgs::PositionYaw& pos_yaw)
{
  tf::vectorMsgToEigen(pos_yaw.position, target_pos_);
  target_yaw_ = pos_yaw.yaw;
}

void PositionControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t level)
{
  updateDynamicParams(cfg);

  pos_controller_->reconfigure(dynamic_params_);
}

void PositionControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  dh_ros::rosWarn("Base state is not received yet.");
}
}  // namespace tobas_multirotor_controller
