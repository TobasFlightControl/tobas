#pragma once

#include <Navio2/RCOutput_Navio2.h>

#include <tobas_tools/constants.hpp>

namespace tobas_real
{
static constexpr char kConfigPath[] = "/home/pi/.config/tobas/config.ini";

static constexpr char kConfigKey_AdcCoef[] = "DEFAULT.adc_coef";
static constexpr char kConfigKey_AccOffsetX[] = "DEFAULT.acc_offset/x";
static constexpr char kConfigKey_AccOffsetY[] = "DEFAULT.acc_offset/y";
static constexpr char kConfigKey_AccOffsetZ[] = "DEFAULT.acc_offset/z";
static constexpr char kConfigKey_MagEllipseAxx[] = "DEFAULT.mag_ellipse/a_xx";
static constexpr char kConfigKey_MagEllipseAyy[] = "DEFAULT.mag_ellipse/a_yy";
static constexpr char kConfigKey_MagEllipseAzz[] = "DEFAULT.mag_ellipse/a_zz";
static constexpr char kConfigKey_MagEllipseAxy[] = "DEFAULT.mag_ellipse/a_xy";
static constexpr char kConfigKey_MagEllipseAyz[] = "DEFAULT.mag_ellipse/a_yz";
static constexpr char kConfigKey_MagEllipseBx[] = "DEFAULT.mag_ellipse/b_x";
static constexpr char kConfigKey_MagEllipseBy[] = "DEFAULT.mag_ellipse/b_y";
static constexpr char kConfigKey_MagEllipseBz[] = "DEFAULT.mag_ellipse/b_z";
static constexpr char kConfigKey_MagEllipseC[] = "DEFAULT.mag_ellipse/c";
static constexpr char kConfigKey_MagEllipseAzx[] = "DEFAULT.mag_ellipse/a_zx";
static constexpr char kConfigKey_RcRollNeutoral[] = "DEFAULT.rc_input/roll/neutoral";
static constexpr char kConfigKey_RcRollLeft[] = "DEFAULT.rc_input/roll/left";
static constexpr char kConfigKey_RcRollRight[] = "DEFAULT.rc_input/roll/right";
static constexpr char kConfigKey_RcPitchNeutoral[] = "DEFAULT.rc_input/pitch/neutoral";
static constexpr char kConfigKey_RcPitchUp[] = "DEFAULT.rc_input/pitch/up";
static constexpr char kConfigKey_RcPitchDown[] = "DEFAULT.rc_input/pitch/down";
static constexpr char kConfigKey_RcYawNeutoral[] = "DEFAULT.rc_input/yaw/neutoral";
static constexpr char kConfigKey_RcYawLeft[] = "DEFAULT.rc_input/yaw/left";
static constexpr char kConfigKey_RcYawRight[] = "DEFAULT.rc_input/yaw/right";
static constexpr char kConfigKey_RcThrustUp[] = "DEFAULT.rc_input/thrust/up";
static constexpr char kConfigKey_RcThrustDown[] = "DEFAULT.rc_input/thrust/down";
static constexpr char kConfigKey_RcToggleUp[] = "DEFAULT.rc_input/toggle/up";
static constexpr char kConfigKey_RcToggleDown[] = "DEFAULT.rc_input/toggle/down";

// https://docs.emlid.com/navio2/dev/adc/
static constexpr uint32_t kPowerModuleVoltageChannel = 2;

static constexpr uint32_t kServoRailSize = 14;
static constexpr double kPwmFrequency = 400.;  // [Hz] PX4のデフォルト値
static constexpr double kPwmMin = 1000.;       // [us]
static constexpr double kPwmMax = 2000.;       // [us]
static constexpr double kPwmDisarm = 900.;     // [us]
static constexpr double kPwmArm = kPwmMin + (kPwmMax - kPwmMin) * tobas::kMotorSpinArm;  // [us]

static constexpr uint32_t kRCInputChannelYaw = 0;
static constexpr uint32_t kRCInputChannelPitch = 1;
static constexpr uint32_t kRCInputChannelThrust = 2;
static constexpr uint32_t kRCInputChannelRoll = 3;
static constexpr uint32_t kRCInputChannelToggle = 4;

static constexpr double kDisarmDuration = 3.;   // [s]
static constexpr double kDisarmInterval = 0.1;  // [s]

static constexpr double kErrorPeriod = 1.;               // [s]
static constexpr double kShowSensorReadingPeriod = 0.5;  // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;    // [s]
static constexpr double kCheckDelayThreshold = 0.02;     // [s]

void setupRCOutput(RCOutput_Navio2& pwm, uint32_t channel);

uint32_t channelFromPin(uint32_t pin);
uint32_t pinFromChannel(uint32_t channel);
}  // namespace tobas_real
