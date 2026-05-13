// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <expected>

#include <rclcpp/wait_for_message.hpp>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_keyboard/keyboard_reader.hpp>
#include <tobas_keyboard/utils.hpp>
#include <tobas_mission_items/mission.hpp>
#include <tobas_ros2_tools/async_node_manager.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_action_client.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_std_tools/range.hpp>

#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>
#include <tobas_mission_msgs/action/execute_mission.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

using namespace std::chrono_literals;

bool takeoff(rclcpp::Node::SharedPtr node)
{
  // アクションクライアントを作成
  tobas::ros2::SyncActionClient<tobas_mission_msgs::action::ExecuteMission> client(node, tobas::action::kExecuteMission);

  // ゴールを作成
  tobas::mission::Takeoff takeoff;
  takeoff.altitude = 3.;
  takeoff.max_speed = 1.5;
  takeoff.max_accel = 4.;
  takeoff.max_jerk = 4.;
  takeoff.altitude_tolerance = 0.5;

  tobas_mission_msgs::msg::MissionItem mission_item;
  mission_item.type = tobas::mission::kTakeoff;
  mission_item.data = tobas::st::toBytes(takeoff);

  tobas_mission_msgs::action::ExecuteMission::Goal goal;
  goal.mission.items.push_back(mission_item);

  // アクションを実行
  if (!client.sendGoalAndWait(goal)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call takeoff action.");
    return false;
  }

  // アクションの成否を確認
  const auto result = client.getResult();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Takeoff action failed: " << result.result->error_message);
    return false;
  }

  return true;
}

std::expected<tobas::kdl::Frame, const char*> waitForCurrentPose(rclcpp::Node::SharedPtr node)
{
  tobas_msgs::OdometryWithCovarianceStamped odom;
  if (!rclcpp::wait_for_message(odom, node, tobas::topic::kOdometry, 1s, tobas::ros2::qos::DefaultQoS())) {
    return std::unexpected("Failed to get the current odometry.");
  }

  return odom.odom.odom.frame;
}

int main(int argc, char** argv)
{
  tobas::ros2::AsyncNodeManager node_manager(argc, argv, "keyboard_teleop");
  const auto node = node_manager.node();

  // 離陸
  if (!takeoff(node)) {
    return EXIT_FAILURE;
  }

  // 現在の位置姿勢を初期コマンドに設定
  const auto init_pose = waitForCurrentPose(node);
  if (!init_pose) {
    RCLCPP_ERROR(node->get_logger(), init_pose.error());
    return EXIT_FAILURE;
  }
  auto cmd_pos = init_pose.value().p;
  auto cmd_yaw = init_pose.value().M.getYaw();

  // 1度のキーボード入力での目標値の変化量を計算
  const auto repeat_interval_ms = tobas::keyboard::getKeyboardRepeatInterval();
  if (!repeat_interval_ms) {
    RCLCPP_ERROR(node->get_logger(), repeat_interval_ms.error());
    return EXIT_FAILURE;
  }
  const auto repeat_interval = static_cast<double>(repeat_interval_ms.value()) * 1e-3;  // [s]
  const auto delta_pos = 3. * repeat_interval;                                          // m/s x s = m
  const auto delta_rot = M_PI_2 * repeat_interval;                                      // rad/s x s = rad

  // 目標値の制限
  const tobas::st::Range<double> x_limit(-10., 10.);
  const tobas::st::Range<double> y_limit(-10., 10.);
  const tobas::st::Range<double> z_limit(-10., 10.);
  const tobas::st::Range<double> yaw_limit(-M_PI, M_PI);

  // キーボードリーダを作成
  tobas::keyboard::KeyboardReader key_reader;

  // コマンドパブリッシャーを登録
  const auto cmd_pub =
    tobas::ros2::createPublisher<tobas_command_msgs::PosVelAccYaw>(node, tobas::topic::kPosVelAccYawCmd);

  // 説明文の表示を開始
  constexpr char kInstructionText[] = "Control your drone!\n"
                                      "---------------------------\n"
                                      "W/S       : Move in the positive/negative direction along X-axis in WCSs\n"
                                      "A/D       : Move in the positive/negative direction along Y-axis in WCSs\n"
                                      "Up/Down   : Move in the positive/negative direction along Z-axis in WCSs\n"
                                      "Left/Right: Turn left/right along Z-axis in WCSs\n"
                                      "Ctrl-C    : Quit\n";
  std::cout << kInstructionText << std::endl;
  const auto instruction_timer = node->create_timer(10s, [&]() { std::cout << kInstructionText << std::endl; });

  while (rclcpp::ok()) {
    // キーボード入力に依ってコマンドを更新
    const auto c = key_reader.readKey();
    if (c < 0) {
      RCLCPP_ERROR(node->get_logger(), "Failed to read keyboard.");
      continue;
    }

    switch (c) {
      case 'w':  // X+
      {
        cmd_pos.x(x_limit.clamp(cmd_pos.x() + delta_pos));
        RCLCPP_INFO_STREAM(node->get_logger(), "[Moving forward] pos[m]: " << cmd_pos << ", yaw[rad]: " << cmd_yaw);
        break;
      }
      case 's':  // X-
      {
        cmd_pos.x(x_limit.clamp(cmd_pos.x() - delta_pos));
        RCLCPP_INFO_STREAM(node->get_logger(), "[Moving backward] pos[m]: " << cmd_pos << ", yaw[rad]: " << cmd_yaw);
        break;
      }
      case 'a':  // Y+
      {
        cmd_pos.y(y_limit.clamp(cmd_pos.y() + delta_pos));
        RCLCPP_INFO_STREAM(node->get_logger(), "[Moving left] pos[m]: " << cmd_pos << ", yaw[rad]: " << cmd_yaw);
        break;
      }
      case 'd':  // Y-
      {
        cmd_pos.y(y_limit.clamp(cmd_pos.y() - delta_pos));
        RCLCPP_INFO_STREAM(node->get_logger(), "[Moving right] pos[m]: " << cmd_pos << ", yaw[rad]: " << cmd_yaw);
        break;
      }
      case tobas::keyboard::UP:  // Z+
      {
        cmd_pos.z(z_limit.clamp(cmd_pos.z() + delta_pos));
        RCLCPP_INFO_STREAM(node->get_logger(), "[Moving up] pos[m]: " << cmd_pos << ", yaw[rad]: " << cmd_yaw);
        break;
      }
      case tobas::keyboard::DOWN:  // Z-
      {
        cmd_pos.z(z_limit.clamp(cmd_pos.z() - delta_pos));
        RCLCPP_INFO_STREAM(node->get_logger(), "[Moving down] pos[m]: " << cmd_pos << ", yaw[rad]: " << cmd_yaw);
        break;
      }
      case tobas::keyboard::LEFT:  // Yaw+
      {
        cmd_yaw = yaw_limit.clamp(cmd_yaw + delta_rot);
        RCLCPP_INFO_STREAM(node->get_logger(), "[Rotating left] pos[m]: " << cmd_pos << ", yaw[rad]: " << cmd_yaw);
        break;
      }
      case tobas::keyboard::RIGHT:  // Yaw-
      {
        cmd_yaw = yaw_limit.clamp(cmd_yaw - delta_rot);
        RCLCPP_INFO_STREAM(node->get_logger(), "[Rotating right] pos[m]: " << cmd_pos << ", yaw[rad]: " << cmd_yaw);
        break;
      }
    }

    // コマンドを発行
    auto cmd = std::make_unique<tobas_command_msgs::PosVelAccYaw>();
    cmd->header.stamp = node->now();
    cmd->priority.data = tobas_command_msgs::msg::Priority::NORMAL;
    cmd->pos = cmd_pos;
    cmd->vel.setZero();
    cmd->acc.setZero();
    cmd->yaw = cmd_yaw;
    cmd_pub->publish(std::move(cmd));
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
