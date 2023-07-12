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
    bs_received_(false),
    cmd_received_(false),
    check_topics_timer_(
      nh_,
      kCheckTopicsTimerPeriod,
      &PositionControllerRos::checkTopicsTimerCb,
      this),
    server_(ros::NodeHandle(kCtrlName))
{
  getRosParams();

  pos_controller_.reset(new PositionController(dynamic_params_));
  vel_yaw_out_.frame_id.data = tobas_msgs::FrameId::GLOBAL;

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
  event_sub_ = nh_.subscribe("event", 1, &PositionControllerRos::eventCb, this);
  base_state_sub_ = nh_.subscribe("base_state", 1, &PositionControllerRos::baseStateCb, this);
  pos_yaw_sub_ =
    nh_.subscribe("command/position_yaw", 1, &PositionControllerRos::targetPositionCb, this);
}

bool PositionControllerRos::isReady()
{
  return bs_received_ && cmd_received_;
}

void PositionControllerRos::initialize()
{
  t_last_cmd_ = ros::Time::now();
}

void PositionControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_.natural_freq = cfg.natural_frequency;
  dynamic_params_.damp_ratio = cfg.damping_ratio;
}

void PositionControllerRos::eventCb(const tobas_msgs::Event& event)
{
  switch (event.data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      ros::shutdown();
      break;
    default:
      break;
  }
}

void PositionControllerRos::baseStateCb(const tobas_msgs::BaseState& bs)
{
  bs_ = bs;

  if (!bs_received_)
  {
    bs_received_ = true;
  }

  if (!is_initialized_)
  {
    if (isReady())
    {
      check_topics_timer_.stop();
      initialize();
      is_initialized_ = true;
      rosInfo("Position controller is ready.");
    }
    return;
  }

  // GCSとの通信が切れるなどして一定時間コマンドを受け取っていない場合は停止
  if (cmd_received_ && (ros::Time::now() - t_last_cmd_).toSec() > kCommandTimeoutThreshold)
  {
    cmd_received_ = false;
    rosInfo(
      "Stop publishing velocity command as no commands have been received for "
      << kCommandTimeoutThreshold << " seconds.");
  }

  if (!cmd_received_)
  {
    return;
  }

  // Update VelocityYaw message
  pos_controller_->update(bs.pose.pos, pos_yaw_in_.pos, vel_yaw_out_.vel);
  vel_yaw_out_.level = pos_yaw_in_.level;
  vel_yaw_out_.yaw = pos_yaw_in_.yaw;  // ヨー角は位置指令をそのまま流す

  // Publish VelocityYaw message
  vel_yaw_pub_.publish(vel_yaw_out_);
}

void PositionControllerRos::targetPositionCb(const tobas_msgs::PositionYaw& pos_yaw)
{
  // 指令位置と現在位置が離れすぎていないか確認
  const auto dist = (pos_yaw.pos - bs_.pose.pos).Norm();
  if (dist > kMaxCommandPositionDeviation)
  {
    rosError(
      "The distance between current position and commanded position is "
      << dist << " m, which exceeds the limit: " << kMaxCommandPositionDeviation
      << " m. The command is ignored.");
    return;
  }

  pos_yaw_in_ = pos_yaw;
  t_last_cmd_ = ros::Time::now();

  if (!cmd_received_)
  {
    cmd_received_ = true;
  }
}

void PositionControllerRos::checkTopicsTimerCb(const ros::TimerEvent&)
{
  if (!bs_received_)
  {
    rosWarn("Base state is not received yet.");
  }

  if (!cmd_received_)
  {
    rosWarn("Command is not received yet.");
  }
}

void PositionControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  updateDynamicParams(cfg);

  pos_controller_->reconfigure(dynamic_params_);
}
}  // namespace tobas_multirotor_controller
