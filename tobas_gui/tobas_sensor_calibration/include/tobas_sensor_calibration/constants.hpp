#pragma once

#include <chrono>

namespace gui
{
namespace sc
{
static constexpr char kPackageName[] = "tobas_sensor_calibration";

static constexpr auto kSetParamTimeout = std::chrono::seconds(3);
}  // namespace sc
}  // namespace gui
