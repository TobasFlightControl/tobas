#pragma once

namespace tobas
{
namespace dxl
{
namespace topic
{
static constexpr char kMotorStates[] = "dynamixel/motor_states";
static constexpr char kPositionCommand[] = "dynamixel/command/joint_positions";
static constexpr char kVelocityCommand[] = "dynamixel/command/joint_velocities";
static constexpr char kEffortCommand[] = "dynamixel/command/joint_efforts";
};  // namespace topic

namespace service
{
static constexpr char kEnableTorques[] = "dynamixel/enable_torques";
}  // namespace service
}  // namespace dxl
}  // namespace tobas
