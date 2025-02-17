#pragma once

#include <cstddef>

namespace real
{
// Path
static constexpr char kTobasResourceDir[] = "/etc/tobas";

// Topics
static constexpr char kSBUSTopic[] = "real/sbus";
static constexpr char kIMUTopic[] = "real/imu";
static constexpr char kMagTopic[] = "real/magnetic_field";
static constexpr char kAirPressureTopic[] = "real/air_pressure";

// RCチャンネル
static constexpr size_t kRcChannelRoll = 0;    // CH1
static constexpr size_t kRcChannelPitch = 1;   // CH2
static constexpr size_t kRcChannelThrot = 2;   // CH3
static constexpr size_t kRcChannelYaw = 3;     // CH4
static constexpr size_t kRcChannelEnable = 4;  // CH5
                                               // CH6: フタバT10J (ヘリ用) のCH6は変更不可のため
static constexpr size_t kRcChannelMode = 6;    // CH7
static constexpr size_t kRcChannelGPSw = 7;    // CH8

namespace handler
{
namespace imu
{
static constexpr char kSetParamSrv[] = "real/set_imu_parameters";

static constexpr char kOffsetXKey[] = "offset_x";
static constexpr char kOffsetYKey[] = "offset_y";
static constexpr char kOffsetZKey[] = "offset_z";
}  // namespace imu

namespace mag
{
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

namespace adc
{
static constexpr char kSetParamSrv[] = "real/set_battery_parameters";

static constexpr char kVoltageKey[] = "voltage_coef";
static constexpr char kCurrentKey[] = "current_coef";
}  // namespace adc

namespace rcin
{
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
static constexpr char kModeAcrobatKey[] = "mode/acrobat";
static constexpr char kModeStabilizeKey[] = "mode/stabilize";
static constexpr char kModeLoiterKey[] = "mode/loiter";
static constexpr char kGPSwOnKey[] = "gpsw/on";
static constexpr char kGPSwOffKey[] = "gpsw/off";
}  // namespace rcin
}  // namespace handler
}  // namespace real
