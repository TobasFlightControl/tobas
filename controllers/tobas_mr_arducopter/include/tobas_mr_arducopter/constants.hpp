#pragma once

#include <cinttypes>

namespace tobas_mr_arducopter
{
// Constants
static constexpr uint32_t kMaxMotors = 255;
static constexpr uint32_t kFdmPortIn = 9002;
static constexpr uint32_t kFdmPortOut = 9003;
static constexpr char kFdmAddr[] = "127.0.0.1";

// Default parameters
static constexpr uint32_t kDefaultMaxConnectionTimeoutCount = 10;
}  // namespace tobas_mr_arducopter
