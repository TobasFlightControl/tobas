#pragma once

#include <chrono>

namespace gui
{
namespace sim
{
static constexpr char kPackageName[] = "tobas_simulation_gui";

static constexpr auto kServiceCallTimeout = std::chrono::seconds(1);
}  // namespace sim
}  // namespace gui
