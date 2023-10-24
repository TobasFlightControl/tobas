#pragma once

#include <cinttypes>

namespace tobas_mr_arducopter
{
// Constants
static constexpr uint32_t kMaxMotors = 255;
static constexpr uint32_t kFdmPortIn = 9002;
static constexpr uint32_t kFdmPortOut = 9003;
static constexpr char kFdmAddr[] = "127.0.0.1";

static constexpr char kArduCopterNS[] = "/arducopter";
static constexpr char kStateTopic[] = "mavros/state";
static constexpr char kLocalPositionPoseTopic[] = "mavros/local_position/pose";
static constexpr char kParamServerStateTopic[] = "param_server_state";

// ArduPilot Parameters: https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.4.1.html
static constexpr char kFrameClass[] = "FRAME_CLASS";
static constexpr char kFrameType[] = "FRAME_TYPE";
static constexpr char kArmingCheck[] = "ARMING_CHECK";

static constexpr uint32_t kMaxConnectionTimeoutCount = 10;

// Default parameters
static constexpr double kWarnPeriod = 3.;  // [s]
}  // namespace tobas_mr_arducopter
