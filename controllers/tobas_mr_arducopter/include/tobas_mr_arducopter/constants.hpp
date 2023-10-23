#pragma once

#include <cinttypes>

namespace tobas_mr_arducopter
{
// Constants
static constexpr uint32_t kMaxMotors = 255;
static constexpr uint32_t kFdmPortIn = 9002;
static constexpr uint32_t kFdmPortOut = 9003;
static constexpr char kFdmAddr[] = "127.0.0.1";

static constexpr char kStateTopic[] = "mavros/state";
static constexpr char kLocalPositionPoseTopic[] = "mavros/local_position/pose";

static constexpr char kParamServerStateTopic[] = "param_server_state";

// Default parameters
static constexpr uint32_t kDefaultMaxConnectionTimeoutCount = 10;
static constexpr double kWarnPeriod = 3.;  // [s]
}  // namespace tobas_mr_arducopter
