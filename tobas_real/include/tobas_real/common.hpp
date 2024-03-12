#pragma once

#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
#include <Navio2/PWM.h>

namespace tobas_real
{
// using ImuDevice = MPU9250;
using ImuDevice = LSM9DS1;

static constexpr char kConfigPath[] = "/home/pi/.config/tobas/config.ini";

static constexpr char kConfigKey_AdcCoef[] = "DEFAULT.adc_coef";

static constexpr char kConfigKey_AccNoiseDensity[] = "DEFAULT.acc_noise_density";
static constexpr char kConfigKey_GyroNoiseDensity[] = "DEFAULT.gyro_noise_density";
static constexpr char kConfigKey_MagNoiseDensity[] = "DEFAULT.mag_noise_density";
static constexpr char kConfigKey_PressureNoiseDensity[] = "DEFAULT.pressure_noise_density";

static constexpr char kConfigKey_AccOffsetX[] = "DEFAULT.acc_offset/x";
static constexpr char kConfigKey_AccOffsetY[] = "DEFAULT.acc_offset/y";
static constexpr char kConfigKey_AccOffsetZ[] = "DEFAULT.acc_offset/z";

static constexpr char kConfigKey_MagEllipseAxx[] = "DEFAULT.mag_ellipse/a_xx";
static constexpr char kConfigKey_MagEllipseAyy[] = "DEFAULT.mag_ellipse/a_yy";
static constexpr char kConfigKey_MagEllipseAzz[] = "DEFAULT.mag_ellipse/a_zz";
static constexpr char kConfigKey_MagEllipseAxy[] = "DEFAULT.mag_ellipse/a_xy";
static constexpr char kConfigKey_MagEllipseAyz[] = "DEFAULT.mag_ellipse/a_yz";
static constexpr char kConfigKey_MagEllipseAzx[] = "DEFAULT.mag_ellipse/a_zx";
static constexpr char kConfigKey_MagEllipseBx[] = "DEFAULT.mag_ellipse/b_x";
static constexpr char kConfigKey_MagEllipseBy[] = "DEFAULT.mag_ellipse/b_y";
static constexpr char kConfigKey_MagEllipseBz[] = "DEFAULT.mag_ellipse/b_z";
static constexpr char kConfigKey_MagEllipseC[] = "DEFAULT.mag_ellipse/c";

static constexpr char kConfigKey_RcRollLeft[] = "DEFAULT.rc_input/roll/left";
static constexpr char kConfigKey_RcRollRight[] = "DEFAULT.rc_input/roll/right";
static constexpr char kConfigKey_RcPitchUp[] = "DEFAULT.rc_input/pitch/up";
static constexpr char kConfigKey_RcPitchDown[] = "DEFAULT.rc_input/pitch/down";
static constexpr char kConfigKey_RcYawLeft[] = "DEFAULT.rc_input/yaw/left";
static constexpr char kConfigKey_RcYawRight[] = "DEFAULT.rc_input/yaw/right";
static constexpr char kConfigKey_RcThrustUp[] = "DEFAULT.rc_input/thrust/up";
static constexpr char kConfigKey_RcThrustDown[] = "DEFAULT.rc_input/thrust/down";
static constexpr char kConfigKey_RcModeProgram[] = "DEFAULT.rc_input/mode/program";
static constexpr char kConfigKey_RcModeStabilize[] = "DEFAULT.rc_input/mode/stabilize";
static constexpr char kConfigKey_RcModeAcrobat[] = "DEFAULT.rc_input/mode/acrobat";
static constexpr char kConfigKey_RcEStopOn[] = "DEFAULT.rc_input/e_stop/on";
static constexpr char kConfigKey_RcEStopOff[] = "DEFAULT.rc_input/e_stop/off";
static constexpr char kConfigKey_RcGPSwOn[] = "DEFAULT.rc_input/gpsw/on";
static constexpr char kConfigKey_RcGPSwOff[] = "DEFAULT.rc_input/gpsw/off";

// https://docs.emlid.com/navio2/dev/adc/
static constexpr size_t kPowerModuleVoltageChannel = 2;

static constexpr size_t kPwmFrequency = 400;  // [Hz] PX4のデフォルト値
static constexpr double kPwmMin = 1000;       // [us]
static constexpr double kPwmMax = 2000;       // [us]
static constexpr double kPwmDisarm = 900;     // [us]

// RCチャンネル
// フタバT10J (ヘリ用) のチャンネル5 (CH6) は修正できないため空けている
// チャンネルは8までを想定．9以上は受信機が対応していないことがある
static constexpr size_t kRcChannelRoll = 0;    // CH1
static constexpr size_t kRcChannelPitch = 1;   // CH2
static constexpr size_t kRcChannelThrust = 2;  // CH3
static constexpr size_t kRcChannelYaw = 3;     // CH4
static constexpr size_t kRcChannelMode = 4;    // CH5
static constexpr size_t kRcChannelEStop = 6;   // CH7
static constexpr size_t kRcChannelGPSw = 7;    // CH8

static constexpr size_t kWaitToRefreshBarometer = 10000;  // [us]
static constexpr double kDisarmInterval = 0.1;            // [s]
static constexpr double kWarnPeriod = 3.;                 // [s]
static constexpr double kErrorPeriod = 1.;                // [s]
static constexpr double kCheckLatencyTimeConst = 1.;      // [s]
static constexpr double kCheckLatencyThreshold = 0.02;    // [s]

static constexpr double kMinAirPressure = 30000.;  // [Pa] 有効な気圧の下限 (エベレスト山頂)
static constexpr double kMaxAirPressure = 120000.;  // [Pa] 有効な気圧の上限 (観測史上最大以上)

void setupRCOutput(PWM& pwm, const size_t& channel);
}  // namespace tobas_real
