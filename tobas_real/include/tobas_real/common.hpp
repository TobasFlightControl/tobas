#pragma once

#include <Navio2/RCOutput_Navio2.h>

namespace tobas_real
{
static constexpr double kGravity = 9.80665;  // [m/s^2]

static const std::string kHomeDirectory = getenv("HOME");
static const std::string kConfigPath = kHomeDirectory + "/.config/tobas/config.ini";

static constexpr char kConfigKey_AdcCoef[] = "DEFAULT.adc_coef";

// https://docs.emlid.com/navio2/dev/adc/
static constexpr uint32_t kPowerModuleVoltageChannel = 2;
static constexpr double kDefaultAdcCoef = 11.3;

static constexpr uint32_t kServoRailSize = 14;
static constexpr double kPwmFrequency = 50.;           // [Hz]
static constexpr double kPwmMin = 1000.;               // [us]
static constexpr double kPwmMax = 2000.;               // [us]
static constexpr double kPwmDisarm = 900.;             // [us]

static constexpr double kDisarmDuration = 3.;          // [s]
static constexpr double kDisarmInterval = 0.1;         // [s]

static constexpr double kErrorPeriod = 1.;             // [s]
static constexpr double kCheckTopicsTimerPeriod = 5.;  // [s]
static constexpr double kCheckDelayThreshold = 0.02;   // [s]

void setupRCOutput(RCOutput_Navio2& pwm, uint32_t channel);

uint32_t channelFromPin(uint32_t pin);
uint32_t pinFromChannel(uint32_t channel);
}  // namespace tobas_real
