#include <eigen_conversions/eigen_msg.h>

#include <dh_ros_tools/rosparam.hpp>

#include <tobas_msgs/VelocityYaw.h>

#include "../include/tobas_multirotor_controller/position_controller_ros.hpp"
#include "../include/tobas_multirotor_controller/constants.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_multirotor_controller
{
PositionControllerRos::PositionControllerRos(ros::NodeHandle nh, ros::NodeHandle pnh)
  : super(nh, pnh),
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

  pos_controller_.configure(dynamic_params_);

  registerPublishers();
  registerSubscribers();

  // Dynamic Reconfigure
  ConfigServer::CallbackType f =
    boost::bind(&PositionControllerRos::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void PositionControllerRos::getRosParams()
{
  dh_ros::getParam(
    nh_, kCtrlName + "/horizontal_natural_frequency", dynamic_params_.hor_natural_freq);
  dh_ros::getParam(nh_, kCtrlName + "/horizontal_damping_ratio", dynamic_params_.hor_damp_ratio);
  dh_ros::getParam(
    nh_, kCtrlName + "/vertical_natural_frequency", dynamic_params_.ver_natural_freq);
  dh_ros::getParam(nh_, kCtrlName + "/vertical_damping_ratio", dynamic_params_.ver_damp_ratio);
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
}

void PositionControllerRos::updateDynamicParams(const ConfigType& cfg)
{
  dynamic_params_.hor_natural_freq = cfg.horizontal_natural_frequency;
  dynamic_params_.hor_damp_ratio = cfg.horizontal_damping_ratio;
  dynamic_params_.ver_natural_freq = cfg.vertical_natural_frequency;
  dynamic_params_.ver_damp_ratio = cfg.vertical_damping_ratio;
}

void PositionControllerRos::eventCb(const tobas_msgs::EventConstPtr& event)
{
  switch (event->data)
  {
    case tobas_msgs::Event::SHUTDOWN:
      nh_.shutdown();
      break;
    default:
      break;
  }
}

void PositionControllerRos::baseStateCb(const tobas_msgs::BaseStateConstPtr& bs)
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

  if (!cmd_received_)
  {
    return;
  }

  const auto vel_yaw_out = boost::make_shared<tobas_msgs::VelocityYaw>();
  vel_yaw_out->frame_id.data = tobas_msgs::FrameId::GLOBAL;

  // Update VelocityYaw message
  pos_controller_.update(bs->pose.pos, pos_yaw_in_->pos, vel_yaw_out->vel);
  vel_yaw_out->level = pos_yaw_in_->level;
  vel_yaw_out->yaw = pos_yaw_in_->yaw;  // ヨー角は位置指令をそのまま流す

  // Publish VelocityYaw message
  vel_yaw_pub_.publish(vel_yaw_out);
}

void PositionControllerRos::targetPositionCb(const tobas_msgs::PositionYawConstPtr& pos_yaw)
{
  if (!bs_received_)
  {
    return;
  }

  // 指令位置と現在位置が離れすぎていないか確認
  const auto dist = (pos_yaw->pos - bs_->pose.pos).Norm();
  if (dist > kMaxCommandPositionDeviation)
  {
    rosError(
      "The distance between current position and commanded position is "
      << dist << " m, which exceeds the limit: " << kMaxCommandPositionDeviation
      << " m. The command is ignored.");
    return;
  }

  pos_yaw_in_ = pos_yaw;

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
    rosInfo("Waiting for Position & Yaw command.");
  }
}

void PositionControllerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  updateDynamicParams(cfg);
  pos_controller_.configure(dynamic_params_);
  rosInfo("Dynamic parameters are updated.");
}
}  // namespace tobas_multirotor_controller
