#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_std_msgs/msg/bool_stamped.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_mission_msgs/action/land.hpp>

#include "../include/tobas_mr_actions/common.hpp"

using namespace std;

class LandServerNode : public tobas::BaseNode
{
  static constexpr double kVerticalSpeed = 0.3;  // [m/s]

  using self = LandServerNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_mission_msgs::action::Land;

public:
  explicit LandServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_std_msgs::msg::BoolStamped::ConstSharedPtr landed_;
  CommandType cmd_;

  ros2::PublisherPtr<CommandType> cmd_pub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_std_msgs::msg::BoolStamped> landed_sub_;
  ros2::ActionServerPtr<ActionType> as_;

  bool disarmRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void landedCb(const tobas_std_msgs::msg::BoolStamped::ConstSharedPtr& landed);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

LandServerNode::LandServerNode(const rclcpp::NodeOptions& options) : super("land_server", options)
{
  cmd_pub_ = createPublisher<CommandType>(tobas::kPosVelYawCmdTopic);

  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);

  as_ = createAction(tobas::kLandAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool LandServerNode::disarmRotors()
{
  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;
  if (!sc.call(req)) {
    TOBAS_ERROR("\"", tobas::kSetArmSrv, "\" is not ready.");
    return false;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    TOBAS_ERROR("Failed to disarm rotors: ", res->message);
    return false;
  }

  return true;
}

void LandServerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void LandServerNode::landedCb(const tobas_std_msgs::msg::BoolStamped::ConstSharedPtr& landed)
{
  landed_ = landed;
}

rclcpp_action::GoalResponse LandServerNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandServerNode::handleCancel(ros2::ActionGoalHandlePtr<ActionType>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LandServerNode::execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle)
{
  TOBAS_INFO("Landing action is requested.");

  // Create result
  const auto result = std::make_shared<ActionType::Result>();

  // Check topics
  if (!odom_) {
    result->message = "Odometry is not received yet.";
    goal_handle->abort(result);
    return;
  }
  if (!landed_) {
    result->message = "Landing state is not received yet.";
    goal_handle->abort(result);
    return;
  }

  // 初期状態
  const auto start_time = get_clock()->now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_z = odom_->frame.p.z();
  const auto start_yaw = odom_->frame.M.getYaw();

  // 高度チェック
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // 着陸検知したらモータを停止して終了
    if (landed_->data) {
      TOBAS_INFO("Landing detected. Stopping motors.");
      if (!disarmRotors()) {
        result->message = "Failed to disarm rotors.";
        goal_handle->abort(result);
        return;
      }

      result->message.clear();
      goal_handle->succeed(result);
      return;
    }

    // コマンドを作成
    const auto t = (get_clock()->now() - start_time).seconds();
    cmd_.level = goal_handle->get_goal()->level;
    cmd_.pos.x(start_x);
    cmd_.pos.y(start_y);
    cmd_.pos.z(start_z - kVerticalSpeed * t);
    cmd_.vel.x(0);
    cmd_.vel.y(0);
    cmd_.vel.z(-kVerticalSpeed);
    cmd_.yaw = start_yaw;

    // コマンドを発行
    const auto cmd_ptr = std::make_unique<CommandType>(cmd_);
    cmd_pub_->publish(move(cmd_));

    // アクション中止の場合は目標速度を0にして終了
    if (goal_handle->is_canceling()) {
      cmd_.vel.setZero();
      cmd_pub_->publish(cmd_);

      result->message.clear();
      goal_handle->canceled(result);
      return;
    }

    rate.sleep();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandServerNode)
