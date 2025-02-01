#pragma once

namespace gazebo
{
// ROS Topics
static constexpr char kThrottleTopicPrefix[] = "gazebo/command/throttle_";
static constexpr char kBatteryGtTopic[] = "gazebo/ground_truth/battery";
static constexpr char kOdometryGtTopic[] = "gazebo/ground_truth/odom";
static constexpr char kWindGtTopic[] = "gazebo/ground_truth/wind";
static constexpr char kRotorStateTopicPrefix[] = "gazebo/rotor_state_";
static constexpr char kRotorStateGtTopicPrefix[] = "gazebo/ground_truth/rotor_state_";
static constexpr char kJointStatesTopic[] = "joint_states";

// ROS Services
static constexpr char kChargeBatterySrv[] = "gazebo/charge_battery";
static constexpr char kGetWindParamsSrv[] = "gazebo/get_wind_parameters";
static constexpr char kSetWindParamsSrv[] = "gazebo/set_wind_parameters";
static constexpr char kGetTetherParamsSrv[] = "gazebo/get_tether_parameters";
static constexpr char kSetTetherParamsSrv[] = "gazebo/set_tether_parameters";
static constexpr char kBreakRotorSrvPrefix[] = "gazebo/break_rotor_";
}  // namespace gazebo
