#pragma once

#include <cinttypes>

namespace tobas_dynamixel_handler
{
// Constants
static constexpr double kDeg2Rad = M_PI / 180;  // degree -> radian
static constexpr double kRpm2Rps = M_PI / 30;   // rpm -> rad/s
static constexpr char kUnavailable[] = "unavailable";
static constexpr char kInactive[] = "inactive";
static constexpr size_t kMinimumLatency = 1;

// Decoding Scale Factors
static constexpr double kDecodeFactorPos = (2 * M_PI) / (1 << 12);
static constexpr double kDecodeFactorVel = 0.229 * kRpm2Rps;
static constexpr double kDecodeFactorAcc = 214.577 * kRpm2Rps * kRpm2Rps;
static constexpr double kDecodeFactorPwm = 100. / 855;
static constexpr double kDecodeFactorVoltage = 0.1;
static constexpr double kDecodeFactorTemp = 1.;

// Default Parameters
static constexpr char kDefaultDeviceName[] = "/dev/ttyUSB0";
static constexpr float kDefaultProtocolVersion = 2.0;
static constexpr size_t kDefaultBaudRate = 57600;      // [Hz]
static constexpr uint8_t kDefaultReturnDelayTime = 0;  // [us]
static constexpr bool kDefaultReadPwm = false;
static constexpr bool kDefaultReadCurrent = false;
static constexpr bool kDefaultReadVelocity = true;
static constexpr bool kDefaultReadPosition = true;
static constexpr bool kDefaultReadVoltage = false;
static constexpr bool kDefaultReadTemperature = false;
static constexpr uint8_t kDefaultId = 1;
static constexpr char kDefaultOperatingMode[] = "position";

// Addresses
static constexpr uint16_t kAddrModelNumber = 0;
static constexpr uint16_t kAddrModelInformation = 2;
static constexpr uint16_t kAddrReturnDelayTime = 9;
static constexpr uint16_t kAddrOperatingMode = 11;
static constexpr uint16_t kAddrTemperatureLimit = 31;
static constexpr uint16_t kAddrMaxVoltageLimit = 32;
static constexpr uint16_t kAddrMinVoltageLimit = 34;
static constexpr uint16_t kAddrPwmLimit = 36;
static constexpr uint16_t kAddrCurrentLimit = 38;
static constexpr uint16_t kAddrAccelerationLimit = 40;
static constexpr uint16_t kAddrVelocityLimit = 44;
static constexpr uint16_t kAddrMaxPositionLimit = 48;
static constexpr uint16_t kAddrMinPositionLimit = 52;
static constexpr uint16_t kAddrToruqeEnable = 64;
static constexpr uint16_t kAddrHardwareErrorStatus = 70;
static constexpr uint16_t kAddrGoalPwm = 100;
static constexpr uint16_t kAddrGoalCurrent = 102;
static constexpr uint16_t kAddrGoalVelocity = 104;
static constexpr uint16_t kAddrGoalPosition = 116;
static constexpr uint16_t kAddrPresentPwm = 124;
static constexpr uint16_t kAddrPresentCurrent = 126;
static constexpr uint16_t kAddrPresentVelocity = 128;
static constexpr uint16_t kAddrPresentPosition = 132;
static constexpr uint16_t kAddrPresentInputVoltage = 144;
static constexpr uint16_t kAddrPresentTemperature = 146;

// Model Number
static constexpr uint16_t kModelNumberXL430W250 = 1060;
static constexpr uint16_t kModelNumberXC430W250 = 1080;
static constexpr uint16_t kModelNumberXM430W350 = 1020;

// Operating Modes
static constexpr uint8_t kControlModeCurrent = 0;
static constexpr uint8_t kControlModeVelocity = 1;
static constexpr uint8_t kControlModePosition = 3;
static constexpr uint8_t kControlModeExtendedPosition = 4;
static constexpr uint8_t kControlModeCurrentBasePosition = 5;
static constexpr uint8_t kControlModePwm = 16;

// Torque Enable
static constexpr uint8_t kTorqueEnable = 1;
static constexpr uint8_t kTorqueDisable = 0;

// Hardware Error Status
static constexpr uint8_t kErrorInputVoltage = 1 << 0;
static constexpr uint8_t kErrorHallSensor = 1 << 1;
static constexpr uint8_t kErrorOverheating = 1 << 2;
static constexpr uint8_t kErrorMotorEncoder = 1 << 3;
static constexpr uint8_t kErrorElectricalShock = 1 << 4;
static constexpr uint8_t kErrorOverload = 1 << 5;

// ROS Topics
static constexpr char kMotorStatesTopic[] = "dynamixel/motor_states";
static constexpr char kJointPositionsCmdTopic[] = "dynamixel/command/joint_positions";
static constexpr char kJointVelocitiesCmdTopic[] = "dynamixel/command/joint_velocities";
static constexpr char kJointEffortsCmdTopic[] = "dynamixel/command/joint_efforts";
}  // namespace tobas_dynamixel_handler
