// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/conversions.hpp"

#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "tobas_rviz_plugin/logger.hpp"

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.conversions");
}

bool jointStateToRobotStateImpl(const sensor_msgs::msg::JointState& joint_state, RobotState& state)
{
  if (joint_state.name.size() != joint_state.position.size()) {
    RCLCPP_ERROR(
      getLogger(),
      "Different number of names and positions in JointState message: %zu, %zu",
      joint_state.name.size(),
      joint_state.position.size());
    return false;
  }

  state.setVariableValues(joint_state);

  return true;
}

bool multiDofJointsToRobotState(const sensor_msgs::msg::MultiDOFJointState& mjs, RobotState& state, const Transforms* tf)
{
  const auto nj = mjs.joint_names.size();
  if (nj != mjs.transforms.size()) {
    RCLCPP_ERROR(getLogger(), "Different number of names, values or frames in MultiDOFJointState message.");
    return false;
  }

  bool error = false;
  Eigen::Isometry3d inv_t;
  bool use_inv_t = false;

  if (nj > 0 && !Transforms::sameFrame(mjs.header.frame_id, state.getRobotModel()->getModelFrame())) {
    if (tf) {
      try {
        // Find the transform that takes the given frame_id to the desired fixed frame.
        const Eigen::Isometry3d& t2fixed_frame = tf->getTransform(mjs.header.frame_id);
        // We update the value of the transform
        // so that it transforms from the known fixed frame to the desired child link.
        inv_t = t2fixed_frame.inverse();
        use_inv_t = true;
      }
      catch (const std::exception& e) {
        RCLCPP_ERROR(getLogger(), "Caught %s", e.what());
        error = true;
      }
    }
    else {
      error = true;
    }

    if (error) {
      RCLCPP_WARN(
        getLogger(),
        "The transform for multi-dof joints was specified in frame '%s' "
        "but it was not possible to transform that to frame '%s'.",
        mjs.header.frame_id.c_str(),
        state.getRobotModel()->getModelFrame().c_str());
    }
  }

  for (size_t i = 0; i < nj; ++i) {
    const auto& joint_name = mjs.joint_names[i];
    if (!state.getRobotModel()->hasJointModel(joint_name)) {
      RCLCPP_WARN(getLogger(), "No joint matching multi-dof joint '%s'.", joint_name.c_str());
      error = true;
      continue;
    }

    auto transform = tf2::transformToEigen(mjs.transforms[i]);

    // If frames do not mach, attempt to transform.
    if (use_inv_t) {
      transform = transform * inv_t;
    }

    state.setJointPositions(joint_name, transform);
  }

  return !error;
}

void robotStateToMultiDofJointState(const RobotState& state, sensor_msgs::msg::MultiDOFJointState& mjs)
{
  const auto& js = state.getRobotModel()->getMultiDOFJointModels();
  mjs.joint_names.clear();
  mjs.transforms.clear();
  for (const auto joint_model : js) {
    geometry_msgs::msg::TransformStamped p;
    if (state.dirtyJointTransform(joint_model)) {
      Eigen::Isometry3d t;
      t.setIdentity();
      joint_model->computeTransform(state.getJointPositions(joint_model), t);
      p = tf2::eigenToTransform(t);
    }
    else {
      p = tf2::eigenToTransform(state.getJointTransform(joint_model));
    }
    mjs.joint_names.push_back(joint_model->getName());
    mjs.transforms.push_back(p.transform);
  }
  mjs.header.frame_id = state.getRobotModel()->getModelFrame();
}

bool robotStateMsgToRobotStateHelper(
  const Transforms* tf,
  const tobas_visualization_msgs::msg::RobotState& robot_state,
  RobotState& state)
{
  const auto& rs = robot_state;

  if (!rs.is_diff && rs.joint_state.name.empty() && rs.multi_dof_joint_state.joint_names.empty()) {
    RCLCPP_ERROR(getLogger(), "Found empty JointState message.");
    return false;
  }

  const auto result1 = jointStateToRobotStateImpl(robot_state.joint_state, state);
  const auto result2 = multiDofJointsToRobotState(robot_state.multi_dof_joint_state, state, tf);
  return result1 || result2;
}
}  // namespace

bool jointStateToRobotState(const sensor_msgs::msg::JointState& joint_state, RobotState& state)
{
  const auto result = jointStateToRobotStateImpl(joint_state, state);
  state.update();
  return result;
}

bool robotStateMsgToRobotState(const tobas_visualization_msgs::msg::RobotState& robot_state, RobotState& state)
{
  const auto result = robotStateMsgToRobotStateHelper(nullptr, robot_state, state);
  state.update();
  return result;
}

bool robotStateMsgToRobotState(
  const Transforms& tf,
  const tobas_visualization_msgs::msg::RobotState& robot_state,
  RobotState& state)
{
  const auto result = robotStateMsgToRobotStateHelper(&tf, robot_state, state);
  state.update();
  return result;
}

void robotStateToRobotStateMsg(const RobotState& state, tobas_visualization_msgs::msg::RobotState& robot_state)
{
  robot_state.is_diff = false;
  robotStateToJointStateMsg(state, robot_state.joint_state);
  robotStateToMultiDofJointState(state, robot_state.multi_dof_joint_state);
}

void robotStateToJointStateMsg(const RobotState& state, sensor_msgs::msg::JointState& joint_state)
{
  const auto& js = state.getRobotModel()->getSingleDOFJointModels();
  joint_state = sensor_msgs::msg::JointState();

  for (const auto joint_model : js) {
    joint_state.name.push_back(joint_model->getName());
    joint_state.position.push_back(state.getVariablePosition(joint_model->getFirstVariableIndex()));
    if (state.hasVelocities()) {
      joint_state.velocity.push_back(state.getVariableVelocity(joint_model->getFirstVariableIndex()));
    }
  }

  // If inconsistent number of velocities are specified, discard them.
  if (joint_state.velocity.size() != joint_state.position.size()) {
    joint_state.velocity.clear();
  }

  joint_state.header.frame_id = state.getRobotModel()->getModelFrame();
}

bool jointTrajPointToRobotState(
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  size_t point_id,
  RobotState& state)
{
  if (trajectory.points.empty() || point_id > trajectory.points.size() - 1) {
    RCLCPP_ERROR(getLogger(), "Invalid point_id.");
    return false;
  }
  if (trajectory.joint_names.empty()) {
    RCLCPP_ERROR(getLogger(), "No joint names specified.");
    return false;
  }

  state.setVariablePositions(trajectory.joint_names, trajectory.points[point_id].positions);
  if (!trajectory.points[point_id].velocities.empty()) {
    state.setVariableVelocities(trajectory.joint_names, trajectory.points[point_id].velocities);
  }
  if (!trajectory.points[point_id].accelerations.empty()) {
    state.setVariableAccelerations(trajectory.joint_names, trajectory.points[point_id].accelerations);
  }
  if (!trajectory.points[point_id].effort.empty()) {
    state.setVariableEffort(trajectory.joint_names, trajectory.points[point_id].effort);
  }

  return true;
}
}  // namespace tobas
