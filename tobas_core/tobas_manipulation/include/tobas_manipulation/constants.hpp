#pragma once

#include <chrono>

namespace tobas
{
namespace manipulation
{
static constexpr auto kAutoResetTimeThresh = std::chrono::milliseconds(500);
}  // namespace manipulation
}  // namespace tobas
