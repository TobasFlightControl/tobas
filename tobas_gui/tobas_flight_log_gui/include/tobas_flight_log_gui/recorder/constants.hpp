#pragma once

#include <chrono>

namespace tobas
{
namespace gui
{
namespace log
{
static constexpr auto kRecordServiceTimeout = std::chrono::seconds(5);
}  // namespace log
}  // namespace gui
}  // namespace tobas
