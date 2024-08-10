#pragma once

namespace gazebo
{
// ROS Topics
static constexpr char kContactStatesTopic[] = "gazebo/contact_states";
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

static constexpr double kWarnPeriod = 1.;              // [s]
static constexpr double kErrorPeriod = 1.;             // [s]
static constexpr double kRotorSpeedSlowdownSim = 10.;  // [-]

static constexpr double kDefaultLatitudeZero = 35.658099;    // [deg] 日本: 北緯35度39分29秒
static constexpr double kDefaultLongitudeZero = 139.741354;  // [deg] 日本: 東経139度44分28秒8759
static constexpr double kDefaultAltitudeZero = 0.;           // [m]
}  // namespace gazebo
