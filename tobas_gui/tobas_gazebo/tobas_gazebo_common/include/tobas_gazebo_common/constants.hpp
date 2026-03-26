#pragma once

namespace tobas
{
namespace gazebo
{
// Gazebo Topics
static constexpr char kGzStatsTopic[] = "/stats";
static constexpr char kGzCameraLookAtTopic[] = "/gui/camera/look_at";
static constexpr char kGzRenderFpsTopic[] = "/gui/render/fps";

// Gazebo Services
static constexpr char kGzMarkerSrv[] = "/marker";

// ROS Topics
static constexpr char kBatteryGtTopic[] = "gazebo/ground_truth/battery";
static constexpr char kOdometryGtTopic[] = "gazebo/ground_truth/odom";
static constexpr char kWindGtTopic[] = "gazebo/ground_truth/wind";
static constexpr char kRotorThrottleCmdTopicNS[] = "gazebo/command/throttle";
static constexpr char kRotorStateTopicNS[] = "gazebo/rotor_state";
static constexpr char kRotorStateGtTopicNS[] = "gazebo/ground_truth/rotor_state";
static constexpr char kJointCommandTopicNS[] = "gazebo/joint_command";
static constexpr char kEngineStateGtTopic[] = "gazebo/ground_truth/engine_state";

// ROS Services
static constexpr char kChargeBatterySrv[] = "gazebo/charge_battery";
static constexpr char kGetWindParamsSrv[] = "gazebo/get_wind_parameters";
static constexpr char kSetWindParamsSrv[] = "gazebo/set_wind_parameters";
static constexpr char kGetTetherParamsSrv[] = "gazebo/get_tether_parameters";
static constexpr char kSetTetherParamsSrv[] = "gazebo/set_tether_parameters";
static constexpr char kAttachSuspenedLoadSrv[] = "gazebo/attach_suspended_load";
static constexpr char kDetachSuspenedLoadSrv[] = "gazebo/detach_suspended_load";
static constexpr char kBreakRotorSrvNS[] = "gazebo/break_rotor";
}  // namespace gazebo
}  // namespace tobas
