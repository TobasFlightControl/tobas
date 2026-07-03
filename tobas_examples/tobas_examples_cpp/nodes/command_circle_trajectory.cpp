// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_math/core.hpp>
#include <tobas_mission_items/mission.hpp>
#include <tobas_ros2_tools/async_node_manager.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_action_client.hpp>
#include <tobas_std_tools/byte.hpp>

#include <tobas_command_msgs/msg/pos_vel_acc_yaw.hpp>
#include <tobas_mission_msgs/action/execute_mission.hpp>

#define ALTITUDE 3.  // [m]

using namespace std::chrono_literals;

bool takeoff(rclcpp::Node::SharedPtr node)
{
  // Create an action client.
  tobas::ros2::SyncActionClient<tobas_mission_msgs::action::ExecuteMission> client(node, tobas::action::kExecuteMission);

  // Create a goal.
  tobas::mission::Takeoff takeoff;
  takeoff.altitude = ALTITUDE;
  takeoff.max_speed = 1.5;
  takeoff.max_accel = 4.;
  takeoff.max_jerk = 4.;
  takeoff.altitude_tolerance = 0.5;

  tobas_mission_msgs::msg::MissionItem mission_item;
  mission_item.type = tobas::mission::kTakeoff;
  mission_item.data = tobas::st::toBytes(takeoff);

  tobas_mission_msgs::action::ExecuteMission::Goal goal;
  goal.mission.items.push_back(mission_item);

  // Execute the action.
  if (!client.sendGoalAndWait(goal)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call takeoff action.");
    return false;
  }

  // Check whether the action succeeded.
  const auto result = client.getResult();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to takeoff: " << result.result->error_message);
    return false;
  }

  return true;
}

bool land(rclcpp::Node::SharedPtr node)
{
  // Create an action client.
  tobas::ros2::SyncActionClient<tobas_mission_msgs::action::ExecuteMission> client(node, tobas::action::kExecuteMission);

  // Create a goal.
  tobas::mission::Land land;
  land.speed = 0.7;

  tobas_mission_msgs::msg::MissionItem mission_item;
  mission_item.type = tobas::mission::kLand;
  mission_item.data = tobas::st::toBytes(land);

  tobas_mission_msgs::action::ExecuteMission::Goal goal;
  goal.mission.items.push_back(mission_item);

  // Execute the action.
  if (!client.sendGoalAndWait(goal)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call land action.");
    return false;
  }

  // Check whether the action succeeded.
  const auto result = client.getResult();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to land: " << result.result->error_message);
    return false;
  }

  return true;
}

bool followCirclePath(rclcpp::Node::SharedPtr node)
{
  constexpr double kRadius = 5.;                                 // [m]
  constexpr double kPeriod = 10.;                                // [s]
  constexpr double kOmega = 2 * M_PI / kPeriod;                  // [rad/s]
  constexpr double kSpeed = kRadius * kOmega;                    // [m/s]
  constexpr double kAccel = kRadius * tobas::math::sqr(kOmega);  // [m/s^2]

  // Create a command publisher.
  const auto pub =
    tobas::ros2::createPublisher<tobas_command_msgs::msg::PosVelAccYaw>(node, tobas::topic::kPosVelAccYawCmd);

  const auto start_time = node->now();

  rclcpp::Rate rate(100., node->get_clock());
  while (rclcpp::ok()) {
    const auto cur_time = node->now();
    const auto t = (cur_time - start_time).seconds();

    if (t > kPeriod * 3) {
      break;
    }

    const auto theta = kOmega * t;
    const auto sin_theta = std::sin(theta);
    const auto cos_theta = std::cos(theta);

    auto cmd = std::make_unique<tobas_command_msgs::msg::PosVelAccYaw>();
    cmd->header.stamp = cur_time;
    cmd->pos.x = kRadius * sin_theta;
    cmd->pos.y = kRadius * (1 - cos_theta);
    cmd->pos.z = ALTITUDE;
    cmd->vel.x = kSpeed * cos_theta;
    cmd->vel.y = kSpeed * sin_theta;
    cmd->vel.z = 0.;
    cmd->acc.x = -kAccel * sin_theta;
    cmd->acc.y = kAccel * cos_theta;
    cmd->acc.z = 0.;
    cmd->yaw = theta;

    pub->publish(std::move(cmd));

    rate.sleep();
  }

  return true;
}

int main(int argc, char** argv)
{
  tobas::ros2::AsyncNodeManager node_manager(argc, argv, "command_circle_trajectory");
  const auto node = node_manager.node();

  // Take off.
  if (!takeoff(node)) {
    return EXIT_FAILURE;
  }

  rclcpp::sleep_for(1s);

  // Fly along a circular path.
  if (!followCirclePath(node)) {
    return EXIT_FAILURE;
  }

  rclcpp::sleep_for(1s);

  // Land.
  if (!land(node)) {
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
