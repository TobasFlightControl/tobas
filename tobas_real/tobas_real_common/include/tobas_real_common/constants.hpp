#pragma once

#include <cstddef>

namespace real
{
// Topics
static constexpr char kIMUTopic[] = "real/imu";
static constexpr char kMagTopic[] = "real/magnetic_field";
static constexpr char kAirPressureTopic[] = "real/air_pressure";

// RCチャンネル
static constexpr size_t kRcChannelRoll = 0;     // CH1
static constexpr size_t kRcChannelPitch = 1;    // CH2
static constexpr size_t kRcChannelThrot = 2;    // CH3
static constexpr size_t kRcChannelYaw = 3;      // CH4
static constexpr size_t kRcChannelEnable = 4;   // CH5
static constexpr size_t kRcChannelKill = 5;     // CH6
static constexpr size_t kRcChannelMode = 6;     // CH7
static constexpr size_t kRcChannelSubMode = 7;  // CH8

namespace handler
{
namespace imu
{
static constexpr char kConfigFileName[] = "imu.ini";
static constexpr char kSetParamSrv[] = "real/set_imu_parameters";

static constexpr char kOffsetXKey[] = "offset_x";
static constexpr char kOffsetYKey[] = "offset_y";
static constexpr char kOffsetZKey[] = "offset_z";
}  // namespace imu

namespace mag
{
static constexpr char kConfigFileName[] = "magnetometer.ini";
static constexpr char kSetParamSrv[] = "real/set_magnetometer_parameters";

static constexpr char kAxxKey[] = "a_xx";
static constexpr char kAyyKey[] = "a_yy";
static constexpr char kAzzKey[] = "a_zz";
static constexpr char kAxyKey[] = "a_xy";
static constexpr char kAyzKey[] = "a_yz";
static constexpr char kAzxKey[] = "a_zx";
static constexpr char kBxKey[] = "b_x";
static constexpr char kByKey[] = "b_y";
static constexpr char kBzKey[] = "b_z";
static constexpr char kCKey[] = "c";
}  // namespace mag

namespace rcin
{
static constexpr char kConfigFileName[] = "rc_input.ini";
static constexpr char kSetParamSrv[] = "real/set_rc_input_parameters";

static constexpr char kRollLeftKey[] = "roll/left";
static constexpr char kRollRightKey[] = "roll/right";
static constexpr char kPitchUpKey[] = "pitch/up";
static constexpr char kPitchDownKey[] = "pitch/down";
static constexpr char kYawLeftKey[] = "yaw/left";
static constexpr char kYawRightKey[] = "yaw/right";
static constexpr char kThrotUpKey[] = "throttle/up";
static constexpr char kThrotDownKey[] = "throttle/down";
static constexpr char kEnableOnKey[] = "enable/on";
static constexpr char kEnableOffKey[] = "enable/off";
static constexpr char kKillOnKey[] = "kill/on";
static constexpr char kKillOffKey[] = "kill/off";
static constexpr char kModeAcrobatKey[] = "mode/acrobat";
static constexpr char kModeStabilizeKey[] = "mode/stabilize";
static constexpr char kModeLoiterKey[] = "mode/loiter";
static constexpr char kSubModeOnKey[] = "sub_mode/on";
static constexpr char kSubModeOffKey[] = "sub_mode/off";
}  // namespace rcin
}  // namespace handler
}  // namespace real
