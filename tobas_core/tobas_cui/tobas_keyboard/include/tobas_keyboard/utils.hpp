#pragma once

#include <cinttypes>
#include <expected>

namespace tobas
{
namespace keyboard
{
/* Get keyboard repeat interval [ms] */
std::expected<uint16_t, const char*> getKeyboardRepeatInterval();
}  // namespace keyboard
}  // namespace tobas
