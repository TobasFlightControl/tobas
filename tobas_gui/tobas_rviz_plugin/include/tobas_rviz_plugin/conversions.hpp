// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tobas_visualization_msgs/msg/robot_state.hpp>

#include "./robot_state.hpp"
#include "./transforms.hpp"

namespace tobas
{
/**
 * @brief Convert a joint state to a Tobas robot state.
 * @param joint_state The input joint state to be converted
 * @param state The resultant Tobas robot state
 * @return True if successful, false if failed for any reason
 */
bool jointStateToRobotState(const sensor_msgs::msg::JointState& joint_state, RobotState& state);

/**
 * @brief Convert a robot state msg (with accompanying extra transforms) to a Tobas robot state.
 * @param tf An instance of a transforms object
 * @param robot_state The input robot state msg
 * @param state The resultant Tobas robot state
 * @return True if successful, false if failed for any reason
 */
bool robotStateMsgToRobotState(
  const Transforms& tf,
  const tobas_visualization_msgs::msg::RobotState& robot_state,
  RobotState& state);

/**
 * @brief Convert a robot state msg (with accompanying extra transforms) to a Tobas robot state.
 * @param robot_state The input robot state msg
 * @param state The resultant Tobas robot state
 * @return True if successful, false if failed for any reason
 */
bool robotStateMsgToRobotState(const tobas_visualization_msgs::msg::RobotState& robot_state, RobotState& state);

/**
 * @brief Convert a Tobas robot state to a robot state message.
 * @param state The input Tobas robot state object
 * @param robot_state The resultant RobotState *message
 */
void robotStateToRobotStateMsg(const RobotState& state, tobas_visualization_msgs::msg::RobotState& robot_state);

/**
 * @brief Convert a Tobas robot state to a joint state message.
 * @param state The input Tobas robot state object
 * @param robot_state The resultant JointState message
 */
void robotStateToJointStateMsg(const RobotState& state, sensor_msgs::msg::JointState& joint_state);

/**
 * @brief Convert a joint trajectory point to a Tobas robot state.
 * @param joint_trajectory The input msg
 * @param point_id The index of the trajectory point in the joint trajectory
 * @param state The resultant Tobas robot state
 * @return True if successful, false if failed for any reason
 */
bool jointTrajPointToRobotState(
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  size_t point_id,
  RobotState& state);
}  // namespace tobas
