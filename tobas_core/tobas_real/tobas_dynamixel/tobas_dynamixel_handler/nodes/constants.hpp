#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <tobas_math/core.hpp>
#include <tobas_math/definitions.hpp>

namespace tobas_dynamixel
{
// Decoding Scale Factors
namespace scale_factor
{
static constexpr double kPosition = M_2PI / (1 << 12);
static constexpr double kVelocity = 0.229 * math::kRpm2Rps;
static constexpr double kAcceleration = 214.577 * math::sqr(math::kRpm2Rps);
static constexpr double kLoad = 0.1;
static constexpr double kPwm = 100. / 855;
static constexpr double kVoltage = 0.1;
static constexpr double kTemperature = 1.;
}  // namespace scale_factor

// Address: https://www.besttechnology.co.jp/modules/knowledge/?X%20Series%20Control%20table#q28d55a1
namespace address
{
static constexpr uint16_t kModelNumber = 0;
static constexpr uint16_t kModelInformation = 2;
static constexpr uint16_t kReturnDelayTime = 9;
static constexpr uint16_t kOperatingMode = 11;
static constexpr uint16_t kTemperatureLimit = 31;
static constexpr uint16_t kMaxVoltageLimit = 32;
static constexpr uint16_t kMinVoltageLimit = 34;
static constexpr uint16_t kPwmLimit = 36;
static constexpr uint16_t kCurrentLimit = 38;
static constexpr uint16_t kAccelerationLimit = 40;
static constexpr uint16_t kVelocityLimit = 44;
static constexpr uint16_t kMaxPositionLimit = 48;
static constexpr uint16_t kMinPositionLimit = 52;
static constexpr uint16_t kToruqeEnable = 64;
static constexpr uint16_t kHardwareErrorStatus = 70;
static constexpr uint16_t kGoalPwm = 100;
static constexpr uint16_t kGoalCurrent = 102;
static constexpr uint16_t kGoalVelocity = 104;
static constexpr uint16_t kGoalPosition = 116;
static constexpr uint16_t kRealtimeTick = 120;
static constexpr uint16_t kMoving = 122;
static constexpr uint16_t kMovingStatus = 123;
static constexpr uint16_t kPresentPwm = 124;
static constexpr uint16_t kPresentCurrent = 126;
static constexpr uint16_t kPresentVelocity = 128;
static constexpr uint16_t kPresentPosition = 132;
static constexpr uint16_t kVelocityTrajectory = 136;
static constexpr uint16_t kPositionTrajectory = 140;
static constexpr uint16_t kPresentInputVoltage = 144;
static constexpr uint16_t kPresentTemperature = 146;
}  // namespace address

// Model Number: https://www.besttechnology.co.jp/modules/knowledge/?X%20Series%20Control%20table#f8895243
namespace model_number
{
static constexpr uint16_t kXC330M077 = 0x4A6;
static constexpr uint16_t kXC330M288 = 0x4B0;
static constexpr uint16_t kXC330M181 = 0x4B0;
// static constexpr uint16_t kXC330M288 = 0x4D8;
static constexpr uint16_t kXC330T181 = 0x4BA;
static constexpr uint16_t kXC330T288 = 0x4C4;
static constexpr uint16_t kXL430W250 = 0x4CE;
static constexpr uint16_t k2XL430W250 = 0x442;
static constexpr uint16_t kXC430W150 = 0x42E;
static constexpr uint16_t kXC430W240 = 0x438;
static constexpr uint16_t k2XC430W250 = 0x488;
static constexpr uint16_t kXM430W210 = 0x406;
static constexpr uint16_t kXH430W210 = 0x3F2;
static constexpr uint16_t kXH430V210 = 0x41A;
static constexpr uint16_t kXD430T210 = 0x3F3;
static constexpr uint16_t kXM430W350 = 0x3FC;
static constexpr uint16_t kXH430W350 = 0x3E8;
static constexpr uint16_t kXH430V350 = 0x410;
static constexpr uint16_t kXD430T350 = 0x3E9;
static constexpr uint16_t kXW430T200 = 0x500;
static constexpr uint16_t kXW430T333 = 0x4F6;
static constexpr uint16_t kXM540W150 = 0x46A;
static constexpr uint16_t kXH540W150 = 0x456;
static constexpr uint16_t kXH540V150 = 0x47E;
static constexpr uint16_t kXM540W270 = 0x460;
static constexpr uint16_t kXH540W270 = 0x44C;
static constexpr uint16_t kXH540V270 = 0x474;
static constexpr uint16_t kXW540T140 = 0x49C;
static constexpr uint16_t kXW540T260 = 0x492;
};  // namespace model_number

// Operating Mode: https://www.besttechnology.co.jp/modules/knowledge/?X%20Series%20Control%20table#j5c7292f
namespace operating_mode
{
static constexpr uint8_t kCurrent = 0;
static constexpr uint8_t kVelocity = 1;
static constexpr uint8_t kPosition = 3;
static constexpr uint8_t kExtendedPosition = 4;
static constexpr uint8_t kCurrentBasePosition = 5;
static constexpr uint8_t kPwm = 16;
}  // namespace operating_mode

// Torque Enable
namespace torque_enable
{
static constexpr uint8_t kEnable = 1;
static constexpr uint8_t kDisable = 0;
}  // namespace torque_enable

// Hardware Error Status: https://www.besttechnology.co.jp/modules/knowledge/?X%20Series%20Control%20table#u710d2bc
namespace hardware_error
{
static constexpr uint8_t kInputVoltage = 1 << 0;
static constexpr uint8_t kHallSensor = 1 << 1;
static constexpr uint8_t kOverheating = 1 << 2;
static constexpr uint8_t kMotorEncoder = 1 << 3;
static constexpr uint8_t kElectricalShock = 1 << 4;
static constexpr uint8_t kOverload = 1 << 5;
};  // namespace hardware_error
}  // namespace tobas_dynamixel
