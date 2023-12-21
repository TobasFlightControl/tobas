#pragma once

namespace tobas_manipulation
{
static constexpr char kVelCtrlJSTopic[] = "joint_velocity_controller/target_joint_states";
static constexpr char kVelCtrlCSTopic[] = "joint_velocity_controller/target_cartesian_states";
static constexpr char kEffortCtrlJSTopic[] = "joint_effort_controller/target_joint_states";
static constexpr char kEffortCtrlCSTopic[] = "joint_effort_controller/target_cartesian_states";

static constexpr double kOdomNotReceivedWarnPeriod = 3.;
}  // namespace tobas_manipulation
