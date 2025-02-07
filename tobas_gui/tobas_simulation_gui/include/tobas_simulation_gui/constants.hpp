#pragma once

#include <chrono>

namespace gui
{
namespace sim
{
static constexpr char kPackageName[] = "tobas_simulation_gui";

// Point Sizes
static constexpr int kTitlePSize = 18;
static constexpr int kLabelPSize = 12;
static constexpr int kBodyPSize = 9;

static constexpr auto kWaitForService = std::chrono::seconds(10);
static constexpr auto kServiceCallTimeout = std::chrono::seconds(1);
}  // namespace sim
}  // namespace gui
