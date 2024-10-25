#pragma once

namespace gazebo
{
// ROS Topics
static constexpr char kThrottleTopicPrefix[] = "gazebo/command/throttle";
static constexpr char kBatteryGtTopic[] = "gazebo/ground_truth/battery";
static constexpr char kOdometryGtTopic[] = "gazebo/ground_truth/odom";
static constexpr char kWindGtTopic[] = "gazebo/ground_truth/wind";
static constexpr char kRotorStateGtTopicPrefix[] = "gazebo/ground_truth/rotor_state";

// ROS Services
static constexpr char kChargeBatterySrv[] = "gazebo/charge_battery";
static constexpr char kGetWindParamsSrv[] = "gazebo/get_wind_parameters";
static constexpr char kSetWindParamsSrv[] = "gazebo/set_wind_parameters";
static constexpr char kGetTetherParamsSrv[] = "gazebo/get_tether_parameters";
static constexpr char kSetTetherParamsSrv[] = "gazebo/set_tether_parameters";
}  // namespace gazebo
