#pragma once

#include <cstdint>
#include <chrono>

namespace tobas
{
// PWM duty period
static constexpr uint16_t kPwmMin = 1000;                     // [us]
static constexpr uint16_t kPwmMax = 2000;                     // [us]
static constexpr uint16_t kPwmMid = (kPwmMin + kPwmMax) / 2;  // [us]

// モータが停止して静止摩擦が発生することを防ぐために，最小スロットル率を設定．
// ESCによっては10%以下だとスロットルと印加電圧が比例しない場合があるため，最低でも10%以上にする．
// cf. https://ardupilot.org/copter/docs/set-motor-range.html
static constexpr double kArmThrot = 0.1;
static constexpr double kMinThrot = 0.;  // The minimum throttle
static constexpr double kMaxThrot = 1.;  // The maximum throttle

// RCInput
static constexpr double kRCInputMin = -1.;
static constexpr double kRCInputMax = 1.;

// Rotor speed control
static constexpr int kMinRotorCtrlGain = 0;
static constexpr int kMaxRotorCtrlGain = 30;

// ROS topics
static constexpr char kTimeReferenceTopic[] = "/shm_driver/time_ref";
static constexpr char kMessageTopic[] = "message";
static constexpr char kDroneTopic[] = "drone";
static constexpr char kKDLTreeTopic[] = "kdl_tree";
static constexpr char kRobotDescriptionTopic[] = "robot_description";
static constexpr char kBatteryTopic[] = "battery";
static constexpr char kBatteryLpfTopic[] = "battery_filtered";
static constexpr char kCPUTopic[] = "cpu";
static constexpr char kRcInputTopic[] = "rc_input";
static constexpr char kIMUTopic[] = "imu";
static constexpr char kImuLpfTopic[] = "imu_filtered";
static constexpr char kMagTopic[] = "magnetic_field";
static constexpr char kAirPressureTopic[] = "air_pressure";
static constexpr char kGNSSTopic[] = "gps";
static constexpr char kGpsOriginTopic[] = "gps_origin";
static constexpr char kLidarTopic[] = "point_cloud";
static constexpr char kExternalOdomTopic[] = "external_odometry";
static constexpr char kRotorSpeedsTopic[] = "rotor_speeds";
static constexpr char kJointStatesTopic[] = "joint_states";
static constexpr char kOdometryTopic[] = "odom";
static constexpr char kEulerTopic[] = "euler";
static constexpr char kWindTopic[] = "wind";
static constexpr char kEventTopic[] = "event";
static constexpr char kLatencyTopic[] = "latency";
static constexpr char kArmingTopic[] = "arming";
static constexpr char kPreArmCheckTopic[] = "prearm_check";
static constexpr char kThrottledTopicNS[] = "throttled";
static constexpr char kRemoteIfaceTopicNS[] = "remote_interface";
// Command
static constexpr char kRotorThrustsCmdTopic[] = "command/rotor_thrusts";
static constexpr char kRotorSpeedsCmdTopic[] = "command/rotor_speeds";
static constexpr char kDeflectionCmdTopic[] = "command/deflections";
static constexpr char kPwmCmdTopic[] = "command/pwm_periods";
static constexpr char kPosVelAccYawCmdTopic[] = "command/pos_vel_acc_yaw";
static constexpr char kRPYThrotCmdTopic[] = "command/rpy_throttle";
static constexpr char kPoseTwistAccelCmdTopic[] = "command/pose_twist_accel";
static constexpr char kSpeedRollDpitchCmdTopic[] = "command/speed_roll_delta_pitch";
static constexpr char kJointPositionsCmdTopic[] = "command/joint_positions";
static constexpr char kJointVelocitiesCmdTopic[] = "command/joint_velocities";
static constexpr char kJointEffortsCmdTopic[] = "command/joint_efforts";
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
static constexpr char kNPCtrlFeedbackTopic[] = "feedback/non_planer_controller";
static constexpr char kFWCtrlFeedbackTopic[] = "feedback/fixed_wing_controller";

// ROS services
static constexpr char kListControllersSrv[] = "controller_manager/list_controllers";
static constexpr char kGetDynamicParamsSrv[] = "get_dynamic_parameters";
static constexpr char kEnableRcOutputSrv[] = "enable_rc_output";
static constexpr char kSetArmSrv[] = "set_arm";
static constexpr char kGetGnssOriginSrv[] = "get_gnss_origin";
static constexpr char kSetGnssOriginSrv[] = "set_gnss_origin";
static constexpr char kROSBagRecordStartSrv[] = "rosbag_record_start";
static constexpr char kROSBagRecordStopSrv[] = "rosbag_record_stop";
static constexpr char kROSBagCleanSrv[] = "rosbag_clean";
static constexpr char kGetRotorControlGainsSrv[] = "get_rotor_control_gains";
static constexpr char kSetRotorControlGainsSrv[] = "set_rotor_control_gains";
static constexpr char kSaveRotorControlGainsSrv[] = "save_rotor_control_gains";

// ROS actions
static constexpr char kTakeoffAction[] = "takeoff_action";
static constexpr char kLandAction[] = "land_action";
static constexpr char kMoveAction[] = "move_action";

// Calibration
static constexpr char kAccelCalibAction[] = "accel_calibration";
static constexpr char kMagCalibSrv[] = "mag_calibration";
static constexpr char kADCCalibAction[] = "adc_calibration";
static constexpr char kRCInputCalibSrv[] = "rcin_calibration";
static constexpr char kESCCalibAction[] = "esc_calibration";

// Controller Manager
namespace controller_manager
{
namespace type
{
static constexpr char kJointStateBroadcaster[] = "joint_state_broadcaster/JointStateBroadcaster";
static constexpr char kForwardCommandController[] = "forward_command_controller/ForwardCommandController";
}  // namespace type
}  // namespace controller_manager

// Node names
static constexpr char kControllerNode[] = "controller";
static constexpr char kObserverNode[] = "observer";

// Frames
static constexpr char kWorldFrame[] = "world";

// Path
static constexpr char kROSBagDirHome[] = "~/.tobas/rosbag";
static constexpr char kROSBagDirRoot[] = "/etc/tobas/rosbag";

// Flight mode
enum flight_mode_t
{
  PROGRAM_MODE,
  STABILIZE_MODE,
  ACROBAT_MODE,
};
static constexpr size_t kNumFlightModes = 3;

enum rc_command_t
{
  PROGRAM,
  POS_VEL_ACC_YAW,
  ROLL_PITCH_YAW_THROTTLE,
  POSE_TWIST_ACCEL,
  SPEED_ROLL_DPITCH,
};

// Console message period
static constexpr double kCheckTopicsMsgPeriod = 5.;  // [s]
static constexpr double kIgnoreCmdMsgPeriod = 1.;    // [s]

// Others
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
}  // namespace tobas
