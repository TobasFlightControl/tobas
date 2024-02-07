#pragma once

#include <cinttypes>
#include <cmath>

#include <tobas_kdl/vector.hpp>

namespace tobas
{
static constexpr double kGravity = 9.80665;  // 重力加速度 [m/s^2]
static constexpr double kDeg2Rad = M_PI / 180;
static constexpr double kRad2Deg = 1 / kDeg2Rad;
static constexpr double kFeetToMeter = 0.3048;
static constexpr double kMeterToFeet = 1 / kFeetToMeter;

static constexpr double kMinAirSpeedThresh = 0.1;  // 空力計算を行う最小の風速 [m/s]
static constexpr size_t kMinPinId = 1;
static constexpr size_t kMaxPinId = 14;

// モータが停止して静止摩擦が発生することを防ぐために，最小スロットル率を設定．
// cf. https://ardupilot.org/copter/docs/set-motor-range.html
static constexpr double kArmThrottle = 0.1;
static constexpr double kMinThrottle = 0.;
static constexpr double kMaxThrottle = 1.;

// ROS parameters
static constexpr char kRobotDescriptionParam[] = "robot_description";

// ROS topics
static constexpr char kTimeReferenceTopic[] = "/shm_driver/time_ref";
static constexpr char kBatteryTopic[] = "battery";
static constexpr char kBatteryLpfTopic[] = "battery_filtered";
static constexpr char kCpuTopic[] = "cpu";
static constexpr char kRcInputTopic[] = "rc_input";
static constexpr char kImuTopic[] = "imu";
static constexpr char kMagTopic[] = "magnetic_field";
static constexpr char kAirPressureTopic[] = "air_pressure";
static constexpr char kGpsTopic[] = "gps";
static constexpr char kLidarTopic[] = "point_cloud";
static constexpr char kExternalOdomTopic[] = "external_odometry";
static constexpr char kRotorSpeedsTopic[] = "rotor_speeds";
static constexpr char kJointStatesTopic[] = "joint_states";
static constexpr char kOdometryTopic[] = "odom";
static constexpr char kWindTopic[] = "wind";
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
static constexpr char kEventTopic[] = "event";
static constexpr char kLatencyTopic[] = "latency";
static constexpr char kThrustCorrectionFactorTopic[] = "thrust_correction_factor";
static constexpr char kControllerFeedbackTopic[] = "controller_feedback";
static constexpr char kObserverFeedbackTopic[] = "observer_feedback";
// tobas_manipulation
static constexpr char kPosCtrlJSTopic[] = "joint_position_controller/target_joint_states";
static constexpr char kPosCtrlCSTopic[] = "joint_position_controller/target_cartesian_states";
static constexpr char kVelCtrlJSTopic[] = "joint_velocity_controller/target_joint_states";
static constexpr char kVelCtrlCSTopic[] = "joint_velocity_controller/target_cartesian_states";
static constexpr char kEffortCtrlJSTopic[] = "joint_effort_controller/target_joint_states";
static constexpr char kEffortCtrlCSTopic[] = "joint_effort_controller/target_cartesian_states";

// ROS services
static constexpr char kListControllersSrv[] = "controller_manager/list_controllers";
static constexpr char kSetupPwmSrv[] = "setup_pwm";
static constexpr char kArmRotorsSrv[] = "arm_rotors";

// ROS actions
static constexpr char kLandingAction[] = "landing_action";
static constexpr char kTakeoffAction[] = "takeoff_action";
static constexpr char kPreArmCheckAction[] = "pre_arm_check";

// Frames
static constexpr char kWorldFrame[] = "world";

static constexpr char kUnknown[] = "unknown";

static constexpr double kCheckTopicsTimerPeriod = 5.;    // [s]
static constexpr double kCommandLevelErrorPeriod = 1.;   // [s]
static constexpr double kWaitForServiceExistence = 1.;   // [s]
static constexpr double kAutoResetTimeThreshold = 0.5;   // [s]
static constexpr double kDisarmDuration = 3.;            // [s]
static constexpr double kStopRotorsSleepTime = 1.;       // [s]
static constexpr double kTakeoffAltitudeThreshold = 1.;  // [m]
static constexpr double kRotSpeedMargin = 1.;            // [rad/s]
static constexpr size_t kStopwatchSamples = 100;

static const KDL::Vector kWorldGravity(0, 0, -kGravity);  // (0, 0, -9.80xxx)
}  // namespace tobas
