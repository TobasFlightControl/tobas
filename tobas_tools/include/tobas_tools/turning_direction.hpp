#pragma once

#include <tobas_std_tools/enum.hpp>

namespace tobas
{
/* Rotating direction. */
DEFINE_NAMED_ENUM(TurningDirection);

static constexpr auto CCW = TurningDirection("CCW", 1);
static constexpr auto CW = TurningDirection("CW", -1);
}  // namespace tobas
