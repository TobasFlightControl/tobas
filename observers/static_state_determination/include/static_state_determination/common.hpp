#pragma once

#include <string>
#include <boost/array.hpp>

namespace static_state_determination
{
static const std::string kActionName = "static_state_determination";
static constexpr double kUpdateRate = 100.;  // [Hz]
static constexpr double kInfoPeriod = 0.5;
}  // namespace static_state_determination
