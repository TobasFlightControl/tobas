#include <tobas_constants/constants.hpp>
#include <tobas_ros2_tools/async_node_manager.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_action_client.hpp>

#include <tobas_command_msgs/msg/pos_vel_yaw.hpp>
#include <tobas_mission_msgs/action/land.hpp>
#include <tobas_mission_msgs/action/takeoff.hpp>

#define ALTITUDE 3.  // [m]

using namespace std::chrono_literals;

bool takeoff(rclcpp::Node::SharedPtr node)
{
  // アクションクライアントを作成
  ros2::SyncActionClient<tobas_mission_msgs::action::Takeoff> client(node, tobas::kTakeoffAction);

  // ゴールを作成
  tobas_mission_msgs::action::Takeoff::Goal goal;
  goal.target_altitude = ALTITUDE;
  goal.max_speed = 1.5;
  goal.max_accel = 4.;
  goal.max_jerk = 4.;
  goal.altitude_tolerance = 0.5;

  // アクションを実行
  if (!client.sendGoalAndWait(goal)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call takeoff action.");
    return false;
  }

  // アクションの成否を確認
  const auto result = client.getResult();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Takeoff action failed: " << result.result->message);
    return false;
  }

  return true;
}

bool land(rclcpp::Node::SharedPtr node)
{
  // アクションクライアントを作成
  ros2::SyncActionClient<tobas_mission_msgs::action::Land> client(node, tobas::kLandAction);

  // ゴールを作成
  tobas_mission_msgs::action::Land::Goal goal;
  goal.speed = 0.7;

  // アクションを実行
  if (!client.sendGoalAndWait(goal)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call land action.");
    return false;
  }

  // アクションの成否を確認
  const auto result = client.getResult();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Land action failed: " << result.result->message);
    return false;
  }

  return true;
}

bool followCirclePath(rclcpp::Node::SharedPtr node)
{
  constexpr double kRadius = 5.;   // [m]
  constexpr double kPeriod = 10.;  // [s]

  // コマンドのパブリッシャーを作成
  const auto pub = ros2::createPublisher<tobas_command_msgs::msg::PosVelYaw>(node, tobas::kPosVelYawCmdTopic);

  const auto start_time = node->now();

  rclcpp::Rate rate(100., node->get_clock());
  while (rclcpp::ok()) {
    const auto cur_time = node->now();
    const auto t = (cur_time - start_time).seconds();

    if (t > kPeriod * 3) {
      break;
    }

    constexpr double kOmega = 2 * M_PI / kPeriod;  // [rad/s]
    constexpr double kSpeed = kRadius * kOmega;    // [m/s]
    const auto theta = kOmega * t;

    auto cmd = std::make_unique<tobas_command_msgs::msg::PosVelYaw>();
    cmd->header.stamp = cur_time;
    cmd->pos.x = kRadius * sin(theta);
    cmd->pos.y = kRadius * (1 - cos(theta));
    cmd->pos.z = ALTITUDE;
    cmd->vel.x = kSpeed * cos(theta);
    cmd->vel.y = kSpeed * sin(theta);
    cmd->vel.z = 0.;
    cmd->yaw = theta;

    pub->publish(std::move(cmd));

    rate.sleep();
  }

  return true;
}

int main(int argc, char** argv)
{
  ros2::AsyncNodeManager node_manager(argc, argv, "command_circle_trajectory");
  const auto node = node_manager.node();

  // 離陸
  if (!takeoff(node)) {
    return EXIT_FAILURE;
  }

  rclcpp::sleep_for(1s);

  // 円周上を周遊
  if (!followCirclePath(node)) {
    return EXIT_FAILURE;
  }

  rclcpp::sleep_for(1s);

  // 着陸
  if (!land(node)) {
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
