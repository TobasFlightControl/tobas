#pragma once

namespace hal
{
static constexpr char kADCTopic[] = "hal/adc";
static constexpr char kSBUSTopic[] = "hal/sbus";
static constexpr char kIMUTopic[] = "hal/imu";
static constexpr char kMagTopic[] = "hal/magnetic_field";
static constexpr char kAirPressureTopic[] = "hal/air_pressure";

static constexpr char kStartMainTimerSrvSuffix[] = "/start_main_timer";
static constexpr char kStopMainTimerSrvSuffix[] = "/stop_main_timer";
}  // namespace hal
