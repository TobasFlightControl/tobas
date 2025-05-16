#pragma once

namespace gazebo
{
// Gazebo Topics
static constexpr char kLookAtPositionTopic[] = "/gui/look_at_position";

// ROS Topics
static constexpr char kBatteryGtTopic[] = "gazebo/ground_truth/battery";
static constexpr char kOdometryGtTopic[] = "gazebo/ground_truth/odom";
static constexpr char kWindGtTopic[] = "gazebo/ground_truth/wind";
static constexpr char kRotorThrottleCmdTopicNS[] = "gazebo/command/throttle";
static constexpr char kRotorStateTopicNS[] = "gazebo/rotor_state";
static constexpr char kRotorStateGtTopicNS[] = "gazebo/ground_truth/rotor_state";
static constexpr char kJointStatesTopic[] = "joint_states";

// ROS Services
static constexpr char kChargeBatterySrv[] = "gazebo/charge_battery";
static constexpr char kGetWindParamsSrv[] = "gazebo/get_wind_parameters";
static constexpr char kSetWindParamsSrv[] = "gazebo/set_wind_parameters";
static constexpr char kGetTetherParamsSrv[] = "gazebo/get_tether_parameters";
static constexpr char kSetTetherParamsSrv[] = "gazebo/set_tether_parameters";
static constexpr char kBreakRotorSrvNS[] = "gazebo/break_rotor";
}  // namespace gazebo
