#pragma once

namespace tobas_dynamixel
{
namespace topic
{
static constexpr char kMotorStates[] = "dynamixel/motor_states";
static constexpr char kJointPosCmd[] = "dynamixel/command/joint_positions";
static constexpr char kJointVelCmd[] = "dynamixel/command/joint_velocities";
static constexpr char kJointEffCmd[] = "dynamixel/command/joint_efforts";
};  // namespace topic

namespace service
{
static constexpr char kEnableTorques[] = "dynamixel/enable_torques";
}  // namespace service
}  // namespace tobas_dynamixel
