#include <dh_std_tools/math.hpp>
#include <dh_ros_tools/console_message.hpp>
#include <dh_ros_tools/exception.hpp>

#include "../include/tobas_mr_arducopter/param_server_ros.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_arducopter
{
ParamServerRos::ParamServerRos(ros::NodeHandle nh, ros::NodeHandle pnh, string name)
  : super(nh, pnh, name), server_(pnh_)
{
  getRosParams();

  param_set_msg_.request.value.integer = 0;
  param_set_sc_ = nh_.serviceClient<mavros_msgs::ParamSet>(kParamSetSrvName);
  if (!param_set_sc_.waitForExistence())
    ROS_THROW_NAMED(name_, "Failed to connect to '" << kParamSetSrvName << "' service server.");

  registerPublishers();
  registerSubscribers();

  ConfigServer::CallbackType f = boost::bind(&self::dynamicReconfigureCb, this, _1, _2);
  server_.setCallback(f);
}

void ParamServerRos::getRosParams()
{
}

void ParamServerRos::registerPublishers()
{
}

void ParamServerRos::registerSubscribers()
{
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

void ParamServerRos::dynamicReconfigureCb(const ConfigType& cfg, uint32_t)
{
  param_id_ = "ATC_ANG_RLL_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_ANG_RLL_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_ANG_RLL_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_ANG_PIT_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_ANG_PIT_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_ANG_PIT_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_ANG_YAW_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_ANG_YAW_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_ANG_YAW_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_RLL_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_RLL_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_RLL_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_PIT_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_PIT_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_PIT_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_YAW_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_YAW_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_YAW_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_RLL_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_RLL_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_RLL_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_PIT_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_PIT_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_PIT_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_YAW_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_YAW_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_YAW_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_RLL_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_RLL_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_RLL_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_PIT_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_PIT_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_PIT_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_YAW_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_YAW_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_YAW_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_RLL_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_RLL_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_RLL_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_PIT_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_PIT_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_PIT_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "ATC_RAT_YAW_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.ATC_RAT_YAW_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.ATC_RAT_YAW_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_POSZ_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_POSZ_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_POSZ_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELZ_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELZ_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELZ_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELZ_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELZ_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELZ_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELZ_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELZ_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELZ_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELZ_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELZ_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELZ_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_ACCZ_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_ACCZ_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_ACCZ_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_ACCZ_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_ACCZ_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_ACCZ_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_ACCZ_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_ACCZ_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_ACCZ_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_ACCZ_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_ACCZ_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_ACCZ_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_POSXY_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_POSXY_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_POSXY_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELXY_P";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELXY_P))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELXY_P;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELXY_I";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELXY_I))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELXY_I;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELXY_D";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELXY_D))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELXY_D;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  param_id_ = "PSC_VELXY_FF";
  if (!params_.contains(param_id_) || !dh_std::isClose(params_[param_id_], cfg.PSC_VELXY_FF))
  {
    param_set_msg_.request.param_id = param_id_;
    param_set_msg_.request.value.real = cfg.PSC_VELXY_FF;
    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
      params_[param_id_] = param_set_msg_.response.value.real;
    else
      rosError(name_, "Failed to set " << param_id_ << ".");
  }

  rosInfo(name_, "Dynamic parameters are updated.");
}
}  // namespace tobas_mr_arducopter
