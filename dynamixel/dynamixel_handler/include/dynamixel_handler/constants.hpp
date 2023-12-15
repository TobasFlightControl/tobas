#pragma once

#include <cinttypes>

namespace dynamixel_handler
{
// Constants
static constexpr double kDeg2Rad = M_PI / 180;  // degree -> radian
static constexpr double kRpm2Rps = M_PI / 30;   // rpm -> rad/s
static constexpr char kUnavailable[] = "unavailable";
static constexpr double kVelocityDecodeFactor = 0.229 * kRpm2Rps;
static constexpr size_t kMinimumLatency = 1;

// Addresses
static constexpr uint16_t kAddrModelNumber = 0;
static constexpr uint16_t kAddrModelInformation = 2;
static constexpr uint16_t kAddrReturnDelayTime = 9;
static constexpr uint16_t kAddrOperatingMode = 11;
static constexpr uint16_t kAddrToruqeEnable = 64;
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
static constexpr uint16_t kModelNumber_XL430W250 = 1060;
static constexpr uint16_t kModelNumber_XC430W250 = 1080;
static constexpr uint16_t kModelNumber_XM430W350 = 1020;

// Operating Modes
static constexpr uint8_t kCurrentControlMode = 0;
static constexpr uint8_t kVelocityControlMode = 1;
static constexpr uint8_t kPositionControlMode = 3;
static constexpr uint8_t kExtendedPositionControlMode = 4;
static constexpr uint8_t kCurrentBasePositionControlMode = 5;
static constexpr uint8_t kPwmControlMode = 16;

// Torque Enable
static constexpr uint8_t kTorqueEnable = 1;
static constexpr uint8_t kTorqueDisable = 0;

// Default Parameters
static constexpr char kDefaultDeviceName[] = "/dev/ttyUSB0";
static constexpr float kDefaultProtocolVersion = 2.0;
static constexpr size_t kDefaultBaudRate = 57600;      // [Hz]
static constexpr uint8_t kDefaultReturnDelayTime = 0;  // [us]
static constexpr uint8_t kDefaultId = 1;
static constexpr char kDefaultOperatingMode[] = "position";
}  // namespace dynamixel_handler
