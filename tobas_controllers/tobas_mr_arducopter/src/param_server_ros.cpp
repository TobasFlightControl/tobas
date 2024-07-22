#include <std_msgs/Bool.h>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/float.hpp>
#include <tobas_std_tools/unordered_map.hpp>
#include <tobas_ros_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>

#include "../include/tobas_mr_arducopter/param_server_ros.hpp"
#include "../include/tobas_mr_arducopter/constants.hpp"

#define RETRY_SLEEP 1

using namespace std;

namespace tobas_mr_arducopter
{
ParamServerRos::ParamServerRos(ros::NodeHandle& nh, ros::NodeHandle& pnh, const string& name)
  : super(nh, pnh, name), server_(pnh_)
{
  getRosParams();

  param_set_sc_ = nh_.serviceClient<mavros_msgs::ParamSet>(kParamSetSrv);
  if (!param_set_sc_.waitForExistence(ros::Duration(tobas::kWaitForServiceExistence)))
    TOBAS_EXIT("Failed to connect to '", kParamSetSrv, "' service server.");

  server_state_pub_ = nh_.advertise<std_msgs::Bool>(kParamServerStateTopic, 1, true);
  state_sub_ = nh_.subscribe(kStateTopic, 1, &self::stateCb, this);
  local_pos_sub_ = nh_.subscribe(kLocalPositionPoseTopic, 1, &self::localPositionCb, this);
  param_updates_sub_ = nh_.subscribe(name + "/parameter_updates", 1, &self::paramUpdatesCb, this);
}

void ParamServerRos::getRosParams()
{
  tobas_ros::getParam(nh_, nh_.getNamespace() + kArduCopterNS + "/frame_class", frame_class_);
  tobas_ros::getParam(nh_, nh_.getNamespace() + kArduCopterNS + "/frame_type", frame_type_);
}

void ParamServerRos::setParams(const dynamic_reconfigure::ConfigConstPtr& cfg)
{
  for (const auto& param : cfg->ints)
  {
    if (tobas_std::contains(ints_, param.name) && ints_[param.name] == param.value)
      continue;

    param_set_msg_.request.param_id = param.name;
    param_set_msg_.request.value.integer = param.value;
    param_set_msg_.request.value.real = 0;

    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
    {
      ints_[param.name] = param.value;
    }
    else
    {
      TOBAS_ERROR("Failed to set ", param.name, ".");
      pnh_.setParam(param.name, ints_[param.name]);
    }
  }

  for (const auto& param : cfg->doubles)
  {
    if (tobas_std::contains(doubles_, param.name) && tobas_std::isClose(doubles_[param.name], param.value))
      continue;

    param_set_msg_.request.param_id = param.name;
    param_set_msg_.request.value.integer = 0;
    param_set_msg_.request.value.real = param.value;

    if (param_set_sc_.call(param_set_msg_) && param_set_msg_.response.success)
    {
      doubles_[param.name] = param.value;
    }
    else
    {
      TOBAS_ERROR("Failed to set ", param.name, ".");
      pnh_.setParam(param.name, doubles_[param.name]);
    }
  }
}

void ParamServerRos::stateCb(const mavros_msgs::StateConstPtr& state)
{
  if (state->system_status != mavros_msgs::CompanionProcessStatus::MAV_STATE_STANDBY)
    return;

  TOBAS_INFO("System status has become MAV_STATE_STANDBY.");
  set_init_config_timer_ = nh_.createTimer(ros::Duration(20), &self::setInitConfigTimerCb, this, true);

  // Unsubscribe
  state_sub_.shutdown();
}

void ParamServerRos::localPositionCb(const geometry_msgs::PoseStampedConstPtr&)
{
  // 状態推定の開始を確認してから初期パラメータの設定を行う
  TOBAS_INFO(
    "First local position is received. The parameter server will be ready in ", kActivationDelayFromFirstPose,
    " seconds.");
  set_init_params_timer_ =
    nh_.createTimer(ros::Duration(kActivationDelayFromFirstPose), &self::setInitParamsTimerCb, this, true);

  // Unsubscribe
  local_pos_sub_.shutdown();
}

void ParamServerRos::paramUpdatesCb(const dynamic_reconfigure::ConfigConstPtr& cfg)
{
  // サーバ起動時に呼ばれる最初のコールバックではパラメータの設定は行わず，値を保持しておく
  if (is_first_update_)
  {
    init_cfg_ = cfg;
    is_first_update_ = false;
    return;
  }

  if (!is_init_params_set_)
  {
    TOBAS_ERROR("Parameter server is not ready.");
    return;
  }

  setParams(cfg);
  TOBAS_INFO("Parameters are updated.");
}

void ParamServerRos::setInitConfigTimerCb(const ros::TimerEvent&)
{
  // FRAME_CLASS
  param_set_msg_.request.param_id = kFrameClass;
  param_set_msg_.request.value.integer = frame_class_;
  param_set_msg_.request.value.real = 0;
  while (!param_set_sc_.call(param_set_msg_) || !param_set_msg_.response.success)
  {
    TOBAS_WARN("Failed to set ", kFrameClass, ". Retrying...");
    ros::Duration(RETRY_SLEEP).sleep();
  }

  // FRAME_TYPE
  param_set_msg_.request.param_id = kFrameType;
  param_set_msg_.request.value.integer = frame_type_;
  param_set_msg_.request.value.real = 0;
  while (!param_set_sc_.call(param_set_msg_) || !param_set_msg_.response.success)
  {
    TOBAS_WARN("Failed to set ", kFrameType, ". Retrying...");
    ros::Duration(RETRY_SLEEP).sleep();
  }

  // ARMING_CHECK
  param_set_msg_.request.param_id = kArmingCheck;
  param_set_msg_.request.value.integer = (1 << 20);  // Disable all
  param_set_msg_.request.value.real = 0;
  while (!param_set_sc_.call(param_set_msg_) || !param_set_msg_.response.success)
  {
    TOBAS_WARN("Failed to set ", kArmingCheck, ". Retrying...");
    ros::Duration(RETRY_SLEEP).sleep();
  }
}

void ParamServerRos::setInitParamsTimerCb(const ros::TimerEvent&)
{
  setParams(init_cfg_);
  is_init_params_set_ = true;
  TOBAS_INFO("Initial parameters are set.");

  // サーバの準備が完了したことをROSメッセージで他のノードに伝える
  const auto server_state = boost::make_shared<std_msgs::Bool>();
  server_state->data = true;
  server_state_pub_.publish(server_state);
}
}  // namespace tobas_mr_arducopter
