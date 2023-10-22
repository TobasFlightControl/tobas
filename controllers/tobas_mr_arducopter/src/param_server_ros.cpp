#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_mr_arducopter/param_server_ros.hpp"
#include "../include/tobas_mr_arducopter/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_arducopter
{
ParamServerRos::ParamServerRos(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name), server_(pnh_)
{
  getRosParams();

  param_set_sc_ = nh_.serviceClient<mavros_msgs::ParamSet>(kParamSetSrvName);
  if (!param_set_sc_.waitForExistence())
    ROS_THROW_NAMED(name_, "Failed to connect to '" << kParamSetSrvName << "' service server.");

  registerPublishers();
  registerSubscribers();
}

void ParamServerRos::getRosParams()
{
}

void ParamServerRos::registerPublishers()
{
}

void ParamServerRos::registerSubscribers()
{
  local_pos_sub_ = nh_.subscribe("mavros/local_position/pose", 1, &self::localPositionCb, this);
  param_updates_sub_ = nh_.subscribe(name_ + "/parameter_updates", 1, &self::paramUpdatesCb, this);
}

void ParamServerRos::setParams(const dynamic_reconfigure::ConfigConstPtr& cfg)
{
  for (const auto& param : cfg->ints)
  {
    if (int_params_[param.name] == param.value)
      continue;

    param_set_msg_.request.param_id = param.name;
    param_set_msg_.request.value.integer = param.value;
    param_set_msg_.request.value.real = 0;

    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
    {
      int_params_[param.name] = param.value;
    }
    else
    {
      rosError(name_, "Failed to set " << param.name << ".");
      pnh_.setParam(param.name, int_params_[param.name]);
    }
  }

  for (const auto& param : cfg->doubles)
  {
    if (dh_std::isClose(double_params_[param.name], param.value))
      continue;

    param_set_msg_.request.param_id = param.name;
    param_set_msg_.request.value.integer = 0;
    param_set_msg_.request.value.real = param.value;

    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
    {
      double_params_[param.name] = param.value;
    }
    else
    {
      rosError(name_, "Failed to set " << param.name << ".");
      pnh_.setParam(param.name, double_params_[param.name]);
    }
  }
}

void ParamServerRos::setParamsMap(const dynamic_reconfigure::ConfigConstPtr& cfg)
{
  for (const auto& param : cfg->ints)
    int_params_[param.name] = param.value;

  for (const auto& param : cfg->doubles)
    double_params_[param.name] = param.value;
}

void ParamServerRos::eventCb(const tobas_msgs::EventConstPtr& event)
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

void ParamServerRos::localPositionCb(const geometry_msgs::PoseStampedConstPtr&)
{
  // 状態推定の開始を確認してから初期パラメータの設定を行う
  if (is_first_local_pos_)
  {
    rosInfo(
      name_, "First local position is received. The parameter server will be ready in "
               << kActivationDelayFromFirstState << " seconds.");
    is_first_local_pos_ = false;
    set_init_params_timer_ = nh_.createTimer(
      ros::Duration(kActivationDelayFromFirstState), &self::setInitParamsTimerCb, this, true);
  }
}

void ParamServerRos::paramUpdatesCb(const dynamic_reconfigure::ConfigConstPtr& cfg)
{
  // サーバ起動時に呼ばれる最初のコールバックではパラメータの設定は行わず，値を保持しておく
  if (is_first_update_)
  {
    init_cfg_ = cfg;
    setParamsMap(cfg);
    is_first_update_ = false;
    return;
  }

  if (!is_init_params_set_)
  {
    rosError(name_, "Parameter server is not ready.");
    return;
  }

  setParams(cfg);
  rosInfo(name_, "Parameters are updated.");
}

void ParamServerRos::setInitParamsTimerCb(const ros::TimerEvent&)
{
  setParams(init_cfg_);
  is_init_params_set_ = true;
  rosInfo(name_, "Initial parameters are set.");
}
}  // namespace tobas_mr_arducopter
