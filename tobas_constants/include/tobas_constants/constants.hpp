#pragma once

#include <chrono>
#include <cstdint>

namespace tobas
{
static constexpr double kMinThrot = 0.;  // The minimum throttle
static constexpr double kMaxThrot = 1.;  // The maximum throttle

// RCInput
static constexpr size_t kMinSbusChannels = 8;
static constexpr size_t kMaxSbusChannels = 16;
static constexpr size_t kMaxNumOfGpsw = kMaxSbusChannels - kMinSbusChannels;
static constexpr double kRcInputMin = -1.;
static constexpr double kRCInputMid = 0.;
static constexpr double kRcInputMax = 1.;

// Rotor speed control
static constexpr int kMinRotorCtrlGain = 0;
static constexpr int kMaxRotorCtrlGain = 30;

// ROS topics
static constexpr char kTimeReferenceTopic[] = "/shm_driver/time_ref";
static constexpr char kMessageTopic[] = "message";
static constexpr char kDroneTopic[] = "drone";
static constexpr char kKdlTreeTopic[] = "kdl_tree";
static constexpr char kRobotDescriptionTopic[] = "robot_description";
static constexpr char kBatteryTopic[] = "battery";
static constexpr char kEngineStateTopic[] = "engine_state";
static constexpr char kCpuTopic[] = "cpu";
static constexpr char kSbusTopic[] = "sbus";
static constexpr char kRcInputTopic[] = "rc_input";
static constexpr char kImuTopic[] = "imu";
static constexpr char kImuRawTopic[] = "imu_raw";
static constexpr char kMagTopic[] = "magnetic_field";
static constexpr char kMagRawTopic[] = "magnetic_field_raw";
static constexpr char kAirPressureTopic[] = "air_pressure";
static constexpr char kAirPressureRawTopic[] = "air_pressure_raw";
static constexpr char kGnssTopic[] = "gnss";
static constexpr char kGnssOriginTopic[] = "gnss_origin";
static constexpr char kLidarTopic[] = "point_cloud";
static constexpr char kExternalOdomTopic[] = "external_odometry";
static constexpr char kRotorStatesTopic[] = "rotor_states";
static constexpr char kRotorLivelinessTopic[] = "rotor_liveliness";
static constexpr char kJointStatesTopic[] = "joint_states_2";
static constexpr char kOdometryTopic[] = "odom";
static constexpr char kEventTopic[] = "event";
static constexpr char kImuSamplingTimeTopic[] = "imu_sampling_time";
static constexpr char kControlLatencyTopic[] = "control_latency";
static constexpr char kArmingTopic[] = "arming";
static constexpr char kPreArmCheckTopic[] = "prearm_check";
static constexpr char kPostArmCheckTopic[] = "postarm_check";
static constexpr char kDisturbanceForceTopic[] = "disturbance_force";
static constexpr char kLandedTopic[] = "landed";
static constexpr char kRosbagStateTopic[] = "rosbag_state";
static constexpr char kThrottledTopicNS[] = "throttled";
static constexpr char kRemoteIfaceTopicNS[] = "remote_interface";
// Low Command
static constexpr char kRotorThrustsCmdTopic[] = "command/rotor_thrusts";
static constexpr char kRotorSpeedsCmdTopic[] = "command/rotor_speeds";
static constexpr char kIcePropulsionSystemCmdTopic[] = "command/ice_propulsion_system";
static constexpr char kDeflectionCmdTopic[] = "command/deflections";
static constexpr char kPwmCmdTopic[] = "command/pwm_periods";
// High Command
static constexpr char kRateCmdTopic[] = "command/rate";
static constexpr char kRateThrottleCmdTopic[] = "command/rate_throttle";
static constexpr char kAngleCmdTopic[] = "command/angle";
static constexpr char kAngleThrottleCmdTopic[] = "command/angle_throttle";
static constexpr char kAccelCmdTopic[] = "command/accel";
static constexpr char kAccelYawCmdTopic[] = "command/accel_yaw";
static constexpr char kPosVelCmdTopic[] = "command/pos_vel";
static constexpr char kPosVelYawCmdTopic[] = "command/pos_vel_yaw";
static constexpr char kSpeedRollDpitchCmdTopic[] = "command/speed_roll_delta_pitch";
// Joint Command
static constexpr char kJointPosCmdTopic[] = "command/joint_positions";
static constexpr char kJointVelCmdTopic[] = "command/joint_velocities";
static constexpr char kJointEffCmdTopic[] = "command/joint_efforts";
// Manipulation
static constexpr char kPosCtrlJSTopic[] = "joint_position_controller/target_joint_states";
static constexpr char kPosCtrlLSTopic[] = "joint_position_controller/target_link_states";
static constexpr char kVelCtrlJSTopic[] = "joint_velocity_controller/target_joint_states";
static constexpr char kVelCtrlLSTopic[] = "joint_velocity_controller/target_link_states";
static constexpr char kEffCtrlJSTopic[] = "joint_effort_controller/target_joint_states";
static constexpr char kEffCtrlLSTopic[] = "joint_effort_controller/target_link_states";
// Feedback
static constexpr char kObsvFeedbackTopic[] = "feedback/observer";
static constexpr char kMRCtrlFeedbackTopic[] = "feedback/multirotor_controller";
static constexpr char kFWCtrlFeedbackTopic[] = "feedback/fixed_wing_controller";

// ROS services
static constexpr char kListControllersSrv[] = "controller_manager/list_controllers";
static constexpr char kGetDynamicParamsSrv[] = "get_dynamic_parameters";
static constexpr char kSetArmSrv[] = "set_arm";
static constexpr char kGetGnssOriginSrv[] = "get_gnss_origin";
static constexpr char kSetGnssOriginSrv[] = "set_gnss_origin";
static constexpr char kRosbagRecordStartSrv[] = "rosbag_record_start";
static constexpr char kRosbagRecordStopSrv[] = "rosbag_record_stop";
static constexpr char kRosbagCleanSrv[] = "rosbag_clean";
static constexpr char kGetRotorControlGainsSrv[] = "get_rotor_control_gains";
static constexpr char kSetRotorControlGainsSrv[] = "set_rotor_control_gains";
static constexpr char kSaveRotorControlGainsSrv[] = "save_rotor_control_gains";

// ROS actions
static constexpr char kTakeoffAction[] = "takeoff_action";
static constexpr char kLandAction[] = "land_action";
static constexpr char kMoveAction[] = "move_action";

// Controller Manager
namespace ctrl_manager
{
namespace type
{
static constexpr char kJointStateBroadcaster[] = "joint_state_broadcaster/JointStateBroadcaster";
static constexpr char kForwardCommandController[] = "forward_command_controller/ForwardCommandController";
}  // namespace type
}  // namespace ctrl_manager

// Node names
namespace node
{
static constexpr char kJointStateBroadcaster[] = "joint_state_broadcaster";
static constexpr char kController[] = "controller";
static constexpr char kObserver[] = "observer";
static constexpr char kRcTeleop[] = "rc_teleop";
static constexpr char kImuPreprocess[] = "imu_preprocess";
}  // namespace node

// PWM keys
namespace pwm
{
static constexpr char kEngineThrottleKey[] = "engine";
}  // namespace pwm

// Frames
static constexpr char kWorldFrame[] = "world";

// Install Path
static constexpr char kROS2JazzyInstallPath[] = "/opt/ros/jazzy";
static constexpr char kTobasInstallPath[] = "/opt/tobas";

// Resource Path
static constexpr char kResourceDirHome[] = "~/Tobas";
static constexpr char kResourceDirRoot[] = "/etc/tobas";
static constexpr char kConfigDirHome[] = "~/Tobas/config";
static constexpr char kConfigDirRoot[] = "/etc/tobas/config";
static constexpr char kColconWSPathHome[] = "~/Tobas/colcon_ws";
static constexpr char kColconWSPathRoot[] = "/etc/tobas/colcon_ws";
static constexpr char kRosbagDirHome[] = "~/Tobas/rosbag";
static constexpr char kRosbagDirRoot[] = "/etc/tobas/rosbag";

// Scale
constexpr auto kAccelScale = 10.;   // [m/s^2]
constexpr auto kDGyroScale = 100.;  // [rad/s^2]

// Console message period
static constexpr double kIgnoreCmdMsgPeriod = 1.;  // [s]

// Others
static constexpr char kTBSExtension[] = ".TBS";
static constexpr char kPropertyServerName[] = "/property_server";
static constexpr char kUnknown[] = "unknown";
static constexpr char kMinimulURDF[] = "<robot name=\"empty\"><link name=\"root\"/></robot>";
static constexpr double kTakeoffAltitudeThreshold = 1.;  // [m]
static constexpr double kRotSpeedMargin = 10.;           // [rad/s]
static constexpr double kMinAirSpeedThresh = 0.1;        // [m/s] 空力計算を行う最小風速
static constexpr double kTypicalInfoPeriod = 5.;         // [s]
static constexpr double kTypicalWarnPeriod = 3.;         // [s]
static constexpr double kTypicalErrorPeriod = 1.;        // [s]
static constexpr auto kPublishArmingPeriod = std::chrono::seconds(1);
static constexpr auto kCheckTopicsPeriod = std::chrono::seconds(5);
static constexpr auto kCommandAutoResetTimeout = std::chrono::milliseconds(200);
static constexpr auto kAutoDisarmTimeout = std::chrono::seconds(10);
static constexpr size_t kMaxRosbagSize = 5'000'000'000UL;  // [byte]
}  // namespace tobas
