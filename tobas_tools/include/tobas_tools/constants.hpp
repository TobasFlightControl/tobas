#pragma once

#include <cinttypes>
#include <cmath>

#include <tobas_kdl/vector.hpp>

namespace tobas
{
// Universal constants
static constexpr double kGravity = 9.80665;  // 重力加速度 [m/s^2]
static constexpr double kDeg2Rad = M_PI / 180;
static constexpr double kRad2Deg = 1 / kDeg2Rad;
static constexpr double kFeetToMeter = 0.3048;
static constexpr double kMeterToFeet = 1 / kFeetToMeter;

// モータが停止して静止摩擦が発生することを防ぐために，最小スロットル率を設定．
// ESCによっては10%以下だとスロットルと印加電圧が比例しない場合があるため，最低でも10%以上にする．
// cf. https://ardupilot.org/copter/docs/set-motor-range.html
static constexpr double kArmThrottle = 0.1;
static constexpr double kMinThrottle = 0.;
static constexpr double kMaxThrottle = 1.;

// RCInput
static constexpr double kRCInputMin = -1.;
static constexpr double kRCInputMax = 1.;

// ROS parameters
static constexpr char kRobotDescriptionParam[] = "robot_description";

// ROS topics
static constexpr char kTimeReferenceTopic[] = "/shm_driver/time_ref";
static constexpr char kBatteryTopic[] = "battery";
static constexpr char kBatteryLpfTopic[] = "battery_filtered";
static constexpr char kCpuTopic[] = "cpu";
static constexpr char kRcInputTopic[] = "rc_input";
static constexpr char kImuTopic[] = "imu";
static constexpr char kImuLpfTopic[] = "imu_filtered";
static constexpr char kMagTopic[] = "magnetic_field";
static constexpr char kAirPressureTopic[] = "air_pressure";
static constexpr char kGpsTopic[] = "gps";
static constexpr char kLidarTopic[] = "point_cloud";
static constexpr char kExternalOdomTopic[] = "external_odometry";
static constexpr char kRotorSpeedsTopic[] = "rotor_speeds";
static constexpr char kJointStatesTopic[] = "joint_states";
static constexpr char kOdometryTopic[] = "odom";
static constexpr char kEulerTopic[] = "euler";
static constexpr char kWindTopic[] = "wind";
static constexpr char kEventTopic[] = "event";
static constexpr char kMessageTopic[] = "message";
static constexpr char kLatencyTopic[] = "latency";
static constexpr char kArmingTopic[] = "arming";
static constexpr char kPreArmCheckTopic[] = "pre_arm_check";
static constexpr char kThrustCorrectionFactorTopic[] = "thrust_correction_factor";
// Command
static constexpr char kPwmCmdTopic[] = "command/pwm";
static constexpr char kThrottlesCmdTopic[] = "command/throttles";
static constexpr char kRotorSpeedsCmdTopic[] = "command/rotor_speeds";
static constexpr char kDeflectionCmdTopic[] = "command/deflections";
static constexpr char kPosVelAccYawCmdTopic[] = "command/pos_vel_acc_yaw";
static constexpr char kPositionYawCmdTopic[] = "command/position_yaw";
static constexpr char kVelocityYawCmdTopic[] = "command/velocity_yaw";
static constexpr char kRpyThrustCmdTopic[] = "command/rpy_thrust";
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
static constexpr char kControllerFeedbackTopic[] = "feedback/controller";
static constexpr char kObserverFeedbackTopic[] = "feedback/observer";

// ROS services
static constexpr char kListControllersSrv[] = "controller_manager/list_controllers";
static constexpr char kEnablePwmSrv[] = "enable_pwm";
static constexpr char kGetArmSrv[] = "get_arm";
static constexpr char kSetArmSrv[] = "set_arm";
static constexpr char kGetGnssOriginSrv[] = "get_gnss_origin";
static constexpr char kSetGnssOriginSrv[] = "set_gnss_origin";
static constexpr char kPreArmCheckSrv[] = "pre_arm_check";
static constexpr char kReloadConfigSrvSuffix[] = "/reload_config";
static constexpr char kStartMainTimerSrvSuffix[] = "/start_main_timer";
static constexpr char kStopMainTimerSrvSuffix[] = "/stop_main_timer";

// ROS actions
static constexpr char kTakeoffAction[] = "takeoff_action";
static constexpr char kLandAction[] = "land_action";
static constexpr char kMoveAction[] = "move_action";

// Frames
static constexpr char kWorldFrame[] = "world";
static constexpr char kNavioFrame[] = "navio";

// Flight mode
static constexpr size_t kFlightModeProgram = 0;
static constexpr size_t kFlightModeStabilize = kFlightModeProgram + 1;
static constexpr size_t kFlightModeAcrobat = kFlightModeStabilize + 1;
static constexpr size_t kNumFlightModes = kFlightModeAcrobat + 1;

// Console message period
static constexpr double kCheckTopicsMsgPeriod = 5.;  // [s]
static constexpr double kIgnoreCmdMsgPeriod = 1.;    // [s]

// Others
static constexpr char kUnknown[] = "unknown";
static constexpr double kWaitForServiceExistence = 1.;   // [s]
static constexpr double kAutoResetTimeThreshold = 1.;    // [s]
static constexpr double kDisarmDuration = 3.;            // [s]
static constexpr double kTakeoffAltitudeThreshold = 1.;  // [m]
static constexpr double kRotSpeedMargin = 10.;           // [rad/s]
static constexpr double kMinAirSpeedThresh = 0.1;        // [m/s] 空力計算を行う最小風速
static constexpr size_t kStopwatchSamples = 100;

static const tobas_kdl::Vector kWorldGravity(0, 0, -kGravity);  // (0, 0, -9.80xxx)
}  // namespace tobas
