#pragma once

#include <cstddef>

namespace tobas_real_ros
{
static constexpr char kPropertyServerFC[] = "/property_server_fc";

static constexpr char kConfigKey_AdcVoltageCoef[] = "adc/voltage_coef";
static constexpr char kConfigKey_AdcCurrentCoef[] = "adc/current_coef";

static constexpr char kConfigKey_AccOffsetX[] = "acc_offset/x";
static constexpr char kConfigKey_AccOffsetY[] = "acc_offset/y";
static constexpr char kConfigKey_AccOffsetZ[] = "acc_offset/z";

static constexpr char kConfigKey_MagEllipseAxx[] = "mag_ellipse/a_xx";
static constexpr char kConfigKey_MagEllipseAyy[] = "mag_ellipse/a_yy";
static constexpr char kConfigKey_MagEllipseAzz[] = "mag_ellipse/a_zz";
static constexpr char kConfigKey_MagEllipseAxy[] = "mag_ellipse/a_xy";
static constexpr char kConfigKey_MagEllipseAyz[] = "mag_ellipse/a_yz";
static constexpr char kConfigKey_MagEllipseAzx[] = "mag_ellipse/a_zx";
static constexpr char kConfigKey_MagEllipseBx[] = "mag_ellipse/b_x";
static constexpr char kConfigKey_MagEllipseBy[] = "mag_ellipse/b_y";
static constexpr char kConfigKey_MagEllipseBz[] = "mag_ellipse/b_z";
static constexpr char kConfigKey_MagEllipseC[] = "mag_ellipse/c";

static constexpr char kConfigKey_RcRollLeft[] = "rc_input/roll/left";
static constexpr char kConfigKey_RcRollRight[] = "rc_input/roll/right";
static constexpr char kConfigKey_RcPitchUp[] = "rc_input/pitch/up";
static constexpr char kConfigKey_RcPitchDown[] = "rc_input/pitch/down";
static constexpr char kConfigKey_RcYawLeft[] = "rc_input/yaw/left";
static constexpr char kConfigKey_RcYawRight[] = "rc_input/yaw/right";
static constexpr char kConfigKey_RcThrottleUp[] = "rc_input/throttle/up";
static constexpr char kConfigKey_RcThrottleDown[] = "rc_input/throttle/down";
static constexpr char kConfigKey_RcModeProgram[] = "rc_input/mode/program";
static constexpr char kConfigKey_RcModeStabilize[] = "rc_input/mode/stabilize";
static constexpr char kConfigKey_RcModeAcrobat[] = "rc_input/mode/acrobat";
static constexpr char kConfigKey_RcEStopOn[] = "rc_input/e_stop/on";
static constexpr char kConfigKey_RcEStopOff[] = "rc_input/e_stop/off";
static constexpr char kConfigKey_RcGPSwOn[] = "rc_input/gpsw/on";
static constexpr char kConfigKey_RcGPSwOff[] = "rc_input/gpsw/off";

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

static constexpr double kWarnPeriod = 3.;   // [s]
static constexpr double kErrorPeriod = 1.;  // [s]
}  // namespace tobas_real_ros
