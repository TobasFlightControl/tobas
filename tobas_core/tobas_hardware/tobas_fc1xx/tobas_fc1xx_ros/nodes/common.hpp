#pragma once

#include <chrono>

namespace fc1xx
{
static constexpr auto kRetryInitializationInterval = std::chrono::seconds(1);
}  // namespace fc1xx
