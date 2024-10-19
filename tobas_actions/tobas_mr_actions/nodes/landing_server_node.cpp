#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/Odometry.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/action/land.hpp>

#include "../include/tobas_mr_actions/common.hpp"

using namespace std;

class LandServerNode : public tobas::BaseNode
{
  static constexpr double kVerticalSpeed = 0.3;         // [m/s]
  static constexpr double kTimeWindow = 5.;             // [s] 高度の変化を見る時間窓の長さ
  static constexpr double kStableAltitudeRange = 0.03;  // [m]

  using self = LandServerNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_msgs::action::Land;

public:
  explicit LandServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  CommandType cmd_;

  ros2::PublisherPtr<CommandType> cmd_pub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::ActionServerPtr<ActionType> as_;

  bool disarmRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

LandServerNode::LandServerNode(const rclcpp::NodeOptions& options) : super("land_server", options)
{
  cmd_pub_ = createPublisher<CommandType>(tobas::kPosVelAccYawCmdTopic);
  odom_sub_ = createSubscriber(path::join(tobas::kThrottledTopicNS, tobas::kOdometryTopic), &self::odomCb, this);
  as_ = createAction(tobas::kLandAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool LandServerNode::disarmRotors()
{
  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;
  if (!sc.call(req))
  {
    TOBAS_ERROR("\"", tobas::kSetArmSrv, "\" is not ready.");
    return false;
  }

  const auto& res = sc.getResponse();
  if (!res->success)
  {
    TOBAS_ERROR("Failed to disarm rotors: ", res->message);
    return false;
  }

  return true;
}

void LandServerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
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

  // Check if odometry is received and is in good status
  if (odom_ == nullptr)
  {
    result->message = "Odometry is not received yet.";
    goal_handle->abort(result);
    return;
  }
  if (odom_->status != tobas_msgs::msg::Odometry::NO_ERROR)
  {
    result->message = "There is a problem with the state estimation.";
    goal_handle->abort(result);
    return;
  }

  // 初期状態
  const auto start_time = get_clock()->now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_z = odom_->frame.p.z();
  const auto start_yaw = kdl::Euler(odom_->frame.M).yaw;

  // 高度データを初期化
  tobas_std::TimestampedBuffer<double> alt_buf(kTimeWindow);
  builtin_interfaces::msg::Time t_last;

  // 高度チェック
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok())
  {
    // オドメトリが更新されたら高度データを追加
    if (odom_->header.stamp != t_last)
    {
      const auto cur_time = ros2::chronoFromRosTime(odom_->header.stamp);
      const auto& altitude = odom_->frame.p.z();
      alt_buf.add(cur_time, altitude);
      t_last = odom_->header.stamp;
    }

    if (alt_buf.isFilled())
    {
      // 一定時間幅の高度が一定の範囲内ならモータを停止して終了
      // FIXME: 着陸判定が甘い．IMU等も利用してより正確に判定しないと危険．
      const auto alt_range = abs(alt_buf.firstValue() - alt_buf.lastValue());
      if (alt_range < kStableAltitudeRange)
      {
        TOBAS_INFO("Landing detected. Stopping motors.");
        if (!disarmRotors())
        {
          result->message = "Failed to disarm rotors.";
          goal_handle->abort(result);
          return;
        }

        result->message.clear();
        goal_handle->succeed(result);
        return;
      }
    }

    // コマンドを作成
    const auto t = (get_clock()->now() - start_time).seconds();
    cmd_.level = goal_handle->get_goal()->level;
    cmd_.frame_id.data = tobas_msgs::msg::FrameId::WORLD;
    cmd_.pos.x(start_x);
    cmd_.pos.y(start_y);
    cmd_.pos.z(start_z - kVerticalSpeed * t);
    cmd_.vel.x(0);
    cmd_.vel.y(0);
    cmd_.vel.z(-kVerticalSpeed);
    cmd_.acc.setZero();
    cmd_.yaw = start_yaw;

    // コマンドを発行
    const auto cmd_ptr = std::make_unique<CommandType>(cmd_);
    cmd_pub_->publish(move(cmd_));

    // アクション中止の場合は目標速度・加速度を0にして終了
    if (goal_handle->is_canceling())
    {
      cmd_.vel.setZero();
      cmd_.acc.setZero();
      cmd_pub_->publish(cmd_);

      result->message.clear();
      goal_handle->canceled(result);
      return;
    }

    rate.sleep();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandServerNode)
