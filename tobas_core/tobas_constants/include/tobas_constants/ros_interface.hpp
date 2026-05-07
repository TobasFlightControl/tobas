// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
static constexpr char kThrottledNS[] = "throttled";
static constexpr char kRemoteIfaceNS[] = "remote_interface";

namespace topic
{
static constexpr char kMessage[] = "message";
static constexpr char kDrone[] = "drone";
static constexpr char kKdlTree[] = "kdl_tree";
static constexpr char kRobotDescription[] = "robot_description";
static constexpr char kBattery[] = "battery";
static constexpr char kEngineState[] = "engine_state";
static constexpr char kCpu[] = "cpu";
static constexpr char kSbus[] = "sbus";
static constexpr char kRcInput[] = "rc_input";
static constexpr char kImuRaw[] = "imu_raw";
static constexpr char kImuFilt[] = "imu_filtered";
static constexpr char kMagneticField[] = "magnetic_field";
static constexpr char kAirPressure[] = "air_pressure";
static constexpr char kGnss[] = "gnss";
static constexpr char kRotorStates[] = "rotor_states";
static constexpr char kRotorLiv[] = "rotor_liveliness";
static constexpr char kJointStates[] = "joint_states_2";
static constexpr char kOdometry[] = "odom";
static constexpr char kTrajSetpoint[] = "trajectory_setpoint";
static constexpr char kExternalPose[] = "external_pose";
static constexpr char kMagRef[] = "magnetic_field_ref";
static constexpr char kGnssOrigin[] = "gnss_origin";
static constexpr char kImuSamplingTime[] = "imu_sampling_time";
static constexpr char kControlLatency[] = "control_latency";
static constexpr char kArming[] = "arming";
static constexpr char kVehicleHealth[] = "vehicle_health";
static constexpr char kUserDefinedHealthStatus[] = "user_defined_health_status";
static constexpr char kVibrationLevel[] = "vibration_level";
static constexpr char kObjectOctomap[] = "object_octomap";
static constexpr char kRepulsiveAccel[] = "repulsive_acceleration";
static constexpr char kDisturbanceForce[] = "disturbance_force";
static constexpr char kLanded[] = "landed";
static constexpr char kRosbagState[] = "rosbag_state";
static constexpr char kHeartbeat[] = "heartbeat";
static constexpr char kRemoteConnection[] = "remote_connection";

// Low Command
static constexpr char kRotorThrustsCmd[] = "command/rotor_thrusts";
static constexpr char kRotorSpeedsCmd[] = "command/rotor_speeds";
static constexpr char kIcePropulsionSystemCmd[] = "command/ice_propulsion_system";
static constexpr char kPwmCmd[] = "command/pwm_periods";

// High Command
static constexpr char kRateCmd[] = "command/rate";
static constexpr char kRateThrotCmd[] = "command/rate_throttle";
static constexpr char kRateThrotVectorCmd[] = "command/rate_throttle_vector";
static constexpr char kAngleCmd[] = "command/angle";
static constexpr char kAngleThrotCmd[] = "command/angle_throttle";
static constexpr char kAngleThrotVectorCmd[] = "command/angle_throttle_vector";
static constexpr char kAccelCmd[] = "command/accel";
static constexpr char kAccelYawCmd[] = "command/accel_yaw";
static constexpr char kAccelPitchYawCmd[] = "command/accel_pitch_yaw";
static constexpr char kPosVelAccCmd[] = "command/pos_vel_acc";
static constexpr char kPosVelAccYawCmd[] = "command/pos_vel_acc_yaw";
static constexpr char kPosVelAccPitchYawCmd[] = "command/pos_vel_acc_pitch_yaw";
static constexpr char kSpeedRollDpitchCmd[] = "command/speed_roll_delta_pitch";

// Joint Command
static constexpr char kJointPosCmd[] = "command/joint_positions";
static constexpr char kJointVelCmd[] = "command/joint_velocities";
static constexpr char kJointEffCmd[] = "command/joint_efforts";

// Manipulation
static constexpr char kPosCtrlJS[] = "joint_position_controller/target_joint_states";
static constexpr char kPosCtrlLS[] = "joint_position_controller/target_link_states";
static constexpr char kVelCtrlJS[] = "joint_velocity_controller/target_joint_states";
static constexpr char kVelCtrlLS[] = "joint_velocity_controller/target_link_states";
static constexpr char kEffCtrlJS[] = "joint_effort_controller/target_joint_states";
static constexpr char kEffCtrlLS[] = "joint_effort_controller/target_link_states";

// Feedback
static constexpr char kObsvFeedback[] = "feedback/observer";
static constexpr char kMRCtrlFeedback[] = "feedback/multirotor_controller";
static constexpr char kFWCtrlFeedback[] = "feedback/fixed_wing_controller";
}  // namespace topic

namespace service
{
static constexpr char kGetDynamicParams[] = "get_dynamic_parameters";
static constexpr char kSetArm[] = "set_arm";
static constexpr char kGetGnssOrigin[] = "get_gnss_origin";
static constexpr char kSetGnssOrigin[] = "set_gnss_origin";
static constexpr char kRosbagRecordStart[] = "rosbag_record_start";
static constexpr char kRosbagRecordStop[] = "rosbag_record_stop";
static constexpr char kRosbagClean[] = "rosbag_clean";
static constexpr char kConfigureImuFilter[] = "configure_imu_filter";
static constexpr char kGetRotorControlGains[] = "get_rotor_control_gains";
static constexpr char kSetRotorControlGains[] = "set_rotor_control_gains";
static constexpr char kSaveRotorControlGains[] = "save_rotor_control_gains";
}  // namespace service

namespace action
{
static constexpr char kExecuteMission[] = "execute_mission";
}  // namespace action
}  // namespace tobas
