#pragma once

#include <chrono>

namespace aso
{
static constexpr auto kRetryInitializationInterval = std::chrono::seconds(1);
}
