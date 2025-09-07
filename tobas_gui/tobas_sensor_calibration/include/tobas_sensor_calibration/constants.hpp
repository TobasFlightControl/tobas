#pragma once

#include <chrono>

namespace gui
{
namespace sc
{
static constexpr char kPackageName[] = "tobas_sensor_calibration";

static constexpr int kTitlePSize = 18;
static constexpr int kLabelPSize = 12;
static constexpr int kBodyPSize = 9;

static constexpr auto kSetParamTimeout = std::chrono::seconds(3);
}  // namespace sc
}  // namespace gui
