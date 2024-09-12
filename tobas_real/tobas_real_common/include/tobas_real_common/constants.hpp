#pragma once

#include <cstddef>

namespace real
{
// RCチャンネル
// フタバT10J (ヘリ用) のチャンネル5 (CH6) は修正できないため空けている
// チャンネルは8までを想定．9以上は受信機が対応していないことがある
static constexpr size_t kRcChannelRoll = 0;      // CH1
static constexpr size_t kRcChannelPitch = 1;     // CH2
static constexpr size_t kRcChannelThrottle = 2;  // CH3
static constexpr size_t kRcChannelYaw = 3;       // CH4
static constexpr size_t kRcChannelMode = 4;      // CH5
static constexpr size_t kRcChannelEStop = 6;     // CH7
static constexpr size_t kRcChannelGPSw = 7;      // CH8

namespace handler
{
static constexpr char kParamName[] = "parameters";

namespace adc
{
static constexpr char kIniPath[] = "~/.config/tobas/adc.ini";

static constexpr size_t kVoltageChannel = 0;
static constexpr size_t kCurrentChannel = 1;
static constexpr size_t kParamSize = 2;

static constexpr char kVoltageKey[] = "voltage_coef";
static constexpr char kCurrentKey[] = "current_coef";
}  // namespace adc

namespace imu
{
static constexpr char kIniPath[] = "~/.config/tobas/accelerometer.ini";

static constexpr size_t kOffsetXChannel = 0;
static constexpr size_t kOffsetYChannel = 1;
static constexpr size_t kOffsetZChannel = 2;
static constexpr size_t kParamSize = 3;

static constexpr char kOffsetXKey[] = "offset_x";
static constexpr char kOffsetYKey[] = "offset_y";
static constexpr char kOffsetZKey[] = "offset_z";
}  // namespace imu

namespace mag
{
static constexpr char kIniPath[] = "~/.config/tobas/magnetometer.ini";

static constexpr size_t kAxxChannel = 0;
static constexpr size_t kAyyChannel = 1;
static constexpr size_t kAzzChannel = 2;
static constexpr size_t kAxyChannel = 3;
static constexpr size_t kAyzChannel = 0;
static constexpr size_t kAzxChannel = 5;
static constexpr size_t kBxChannel = 6;
static constexpr size_t kByChannel = 7;
static constexpr size_t kBzChannel = 8;
static constexpr size_t kCChannel = 9;
static constexpr size_t kParamSize = 10;

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
static constexpr char kIniPath[] = "~/.config/tobas/rc_input.ini";

static constexpr size_t kRollLeftChannel = 0;
static constexpr size_t kRollRightChannel = 1;
static constexpr size_t kPitchUpChannel = 2;
static constexpr size_t kPitchDownChannel = 3;
static constexpr size_t kYawLeftChannel = 4;
static constexpr size_t kYawRightChannel = 5;
static constexpr size_t kThrotUpChannel = 6;
static constexpr size_t kThrotDownChannel = 7;
static constexpr size_t kModeProgramChannel = 8;
static constexpr size_t kModeStabilizeChannel = 9;
static constexpr size_t kModeAcrobatChannel = 10;
static constexpr size_t kEStopOnChannel = 11;
static constexpr size_t kEStopOffChannel = 12;
static constexpr size_t kGPSwOnChannel = 13;
static constexpr size_t kGPSwOffChannel = 14;
static constexpr size_t kParamSize = 15;

static constexpr char kRollLeftKey[] = "roll/left";
static constexpr char kRollRightKey[] = "roll/right";
static constexpr char kPitchUpKey[] = "pitch/up";
static constexpr char kPitchDownKey[] = "pitch/down";
static constexpr char kYawLeftKey[] = "yaw/left";
static constexpr char kYawRightKey[] = "yaw/right";
static constexpr char kThrotUpKey[] = "throttle/up";
static constexpr char kThrotDownKey[] = "throttle/down";
static constexpr char kModeProgramKey[] = "mode/program";
static constexpr char kModeStabilizeKey[] = "mode/stabilize";
static constexpr char kModeAcrobatKey[] = "mode/acrobat";
static constexpr char kEStopOnKey[] = "e_stop/on";
static constexpr char kEStopOffKey[] = "e_stop/off";
static constexpr char kGPSwOnKey[] = "gpsw/on";
static constexpr char kGPSwOffKey[] = "gpsw/off";
}  // namespace rcin
}  // namespace handler
}  // namespace real
