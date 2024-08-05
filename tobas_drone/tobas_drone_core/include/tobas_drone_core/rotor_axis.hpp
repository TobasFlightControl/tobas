#pragma once

#include <tobas_std_tools/enum.hpp>

namespace tobas
{
/* Rotating axis types. */
DEFINE_NAMED_ENUM(RotorAxis);

static constexpr auto X_POSITIVE = RotorAxis("X_POSITIVE", 0);
static constexpr auto Z_POSITIVE = RotorAxis("Z_POSITIVE", 1);
static constexpr auto UNKNOWN = RotorAxis("UNKNOWN", 2);
}  // namespace tobas
