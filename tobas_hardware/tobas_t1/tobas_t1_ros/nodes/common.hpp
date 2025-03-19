#pragma once

#include <chrono>

namespace t1
{
static constexpr auto kRetryInitializationInterval = std::chrono::seconds(1);
}
