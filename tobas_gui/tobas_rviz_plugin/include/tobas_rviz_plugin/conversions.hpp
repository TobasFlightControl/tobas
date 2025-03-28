#pragma once

#include "./robot_state.hpp"
#include "./transforms.hpp"

#include <tobas_visualization_msgs/msg/robot_state.hpp>

namespace tobas
{
/**
 * @brief Convert a joint state to a Tobas robot state
 * @param joint_state The input joint state to be converted
 * @param state The resultant Tobas robot state
 * @return True if successful, false if failed for any reason
 */
bool jointStateToRobotState(const sensor_msgs::msg::JointState& joint_state, RobotState& state);

/**
 * @brief Convert a robot state msg (with accompanying extra transforms) to a Tobas robot state
 * @param tf An instance of a transforms object
 * @param robot_state The input robot state msg
 * @param state The resultant Tobas robot state
 * @param copy_attached_bodies Flag to include attached objects in robot state copy
 * @return True if successful, false if failed for any reason
 */
bool robotStateMsgToRobotState(
  const Transforms& tf,
  const tobas_visualization_msgs::msg::RobotState& robot_state,
  RobotState& state,
  bool copy_attached_bodies = true);

/**
 * @brief Convert a robot state msg (with accompanying extra transforms) to a Tobas robot state
 * @param robot_state The input robot state msg
 * @param state The resultant Tobas robot state
 * @param copy_attached_bodies Flag to include attached objects in robot state copy
 * @return True if successful, false if failed for any reason
 */
bool robotStateMsgToRobotState(
  const tobas_visualization_msgs::msg::RobotState& robot_state,
  RobotState& state,
  bool copy_attached_bodies = true);

/**
 * @brief Convert a Tobas robot state to a robot state message
 * @param state The input Tobas robot state object
 * @param robot_state The resultant RobotState *message
 * @param copy_attached_bodies Flag to include attached objects in robot state copy
 */
void robotStateToRobotStateMsg(
  const RobotState& state,
  tobas_visualization_msgs::msg::RobotState& robot_state,
  bool copy_attached_bodies = true);

/**
 * @brief Convert AttachedBodies to AttachedCollisionObjects
 * @param attached_bodies The input Tobas attached body objects
 * @param attached_collision_objs The resultant AttachedCollisionObject messages
 */
void attachedBodiesToAttachedCollisionObjectMsgs(
  const std::vector<const AttachedBody*>& attached_bodies,
  std::vector<tobas_visualization_msgs::msg::AttachedCollisionObject>& attached_collision_objs);
/**
 * @brief Convert a Tobas robot state to a joint state message
 * @param state The input Tobas robot state object
 * @param robot_state The resultant JointState message
 */
void robotStateToJointStateMsg(const RobotState& state, sensor_msgs::msg::JointState& joint_state);

/**
 * @brief Convert a joint trajectory point to a Tobas robot state
 * @param joint_trajectory The input msg
 * @param point_id The index of the trajectory point in the joint trajectory.
 * @param state The resultant Tobas robot state
 * @return True if successful, false if failed for any reason
 */
bool jointTrajPointToRobotState(
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  std::size_t point_id,
  RobotState& state);

/**
 * @brief Convert a Tobas robot state to common separated values (CSV) on a single line that is
 *        outputted to a stream e.g. for file saving
 * @param state - The input Tobas robot state object
 * @param out - a file stream, or any other stream
 * @param include_header - flag to prefix the output with a line of joint names.
 * @param separator - allows to override the comma separator with any symbol, such as a white space
 */
void robotStateToStream(
  const RobotState& state,
  std::ostream& out,
  bool include_header = true,
  const std::string& separator = ",");

/**
 * @brief Convert a Tobas robot state to common separated values (CSV) on a single line that is
 *        outputted to a stream e.g. for file saving. This version can order by joint model groups
 * @param state - The input Tobas robot state object
 * @param out - a file stream, or any other stream
 * @param joint_group_ordering - output joints based on ordering of joint groups
 * @param include_header - flag to prefix the output with a line of joint names.
 * @param separator - allows to override the comma separator with any symbol, such as a white space
 */
void robotStateToStream(
  const RobotState& state,
  std::ostream& out,
  const std::vector<std::string>& joint_groups_ordering,
  bool include_header = true,
  const std::string& separator = ",");

/**
 * @brief Convert a string of joint values from a file (CSV) or input source into a RobotState
 * @param state - the output Tobas robot state object
 * @param line - the input string of joint values
 * @param separator - allows to override the comma separator with any symbol, such as a white space
 * @return true on success
 */
void streamToRobotState(RobotState& state, const std::string& line, const std::string& separator = ",");
}  // namespace tobas
