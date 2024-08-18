#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_ros2_tools/simple_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/PosVelAccYaw.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/action/land.hpp>

using namespace std;

class LandActionServerNode : public tobas::BaseNode
{
  static constexpr double kUpdateRate = 100.;           // [Hz]
  static constexpr double kVerticalSpeed = 0.3;         // [m/s]
  static constexpr double kTimeWindow = 5.;             // [s] 高度の変化を見る時間窓の長さ
  static constexpr double kStableAltitudeRange = 0.03;  // [m]

  using self = LandActionServerNode;
  using super = tobas::BaseNode;

  using ActionType = tobas_msgs::action::Land;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionType>;
  using GoalHandlePtr = shared_ptr<GoalHandle>;

public:
  explicit LandActionServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool is_action_running_ = false;
  tobas_std::TimestampedBuffer<double> alt_buf_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;

  PublisherPtr<tobas_msgs::PosVelAccYaw> cmd_pub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;

  rclcpp_action::Server<ActionType>::SharedPtr as_;

  bool disarmRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(GoalHandlePtr goal_handle);
  void execute(GoalHandlePtr goal_handle);
};

LandActionServerNode::LandActionServerNode(const rclcpp::NodeOptions& options)
  : super("land_action_server", options), alt_buf_(kTimeWindow)
{
  cmd_pub_ = createPublisher<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);

  as_ = createAction(tobas::kLandAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool LandActionServerNode::disarmRotors()
{
  ros2::SimpleServiceClient<tobas_msgs::srv::SetArm> set_arm_sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  if (!set_arm_sc.call(req))
  {
    TOBAS_ERROR("Failed to call \"", tobas::kSetArmSrv, "\" service.");
    return false;
  }

  const auto& res = set_arm_sc.getResponse();
  if (!res->success)
  {
    TOBAS_ERROR("Failed to disarm rotors: ", res->message);
    return false;
  }

  return true;
}

void LandActionServerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (odom->status != tobas_msgs::msg::Odometry::NO_ERROR)
    return;

  odom_ = odom;

  if (!is_action_running_)
    return;

  // 現在の時刻と高度を履歴に追加
  const auto cur_time = ros2::chronoFromRosTime(odom->header.stamp);
  const auto& altitude = odom->frame.p.z();
  alt_buf_.add(cur_time, altitude);
}

rclcpp_action::GoalResponse
LandActionServerNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandActionServerNode::handleCancel(GoalHandlePtr)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LandActionServerNode::execute(GoalHandlePtr goal_handle)
{
  TOBAS_INFO("Landing action is executing.");

  const auto result = std::make_shared<ActionType::Result>();

  // オドメトリが発行されていることを確認
  if (odom_ == nullptr)
  {
    result->message = "Odometry is not received yet.";
    goal_handle->abort(result);
  }

  // 高度のバッファを初期化
  alt_buf_.clear();

  // 初期状態
  const auto start_time = get_clock()->now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_z = odom_->frame.p.z();
  const auto start_yaw = kdl::Euler(odom_->frame.M).yaw;

  // Now the action is running
  is_action_running_ = true;

  // 高度チェック
  rclcpp::Rate rate(kUpdateRate);
  while (rclcpp::ok())
  {
    if (alt_buf_.isFilled())
    {
      // 一定時間幅の高度が一定の範囲内ならモータを停止して終了
      // FIXME: 着陸判定が甘い．IMU等も利用してより正確に判定しないと危険．
      const auto alt_range = abs(alt_buf_.firstValue() - alt_buf_.lastValue());
      if (alt_range < kStableAltitudeRange)
      {
        TOBAS_INFO("Landing detected. Stopping motors.");
        is_action_running_ = false;
        if (!disarmRotors())
        {
          result->message = "Failed to disarm rotors.";
          goal_handle->abort(result);
          break;
        }

        result->message.clear();
        goal_handle->succeed(result);
        break;
      }
    }

    // コマンドを作成
    tobas_msgs::PosVelAccYaw cmd;
    const auto t = (get_clock()->now() - start_time).seconds();
    cmd.level = goal_handle->get_goal()->level;
    cmd.frame_id.data = tobas_msgs::msg::FrameId::WORLD;
    cmd.pos.x(start_x);
    cmd.pos.y(start_y);
    cmd.pos.z(start_z - kVerticalSpeed * t);
    cmd.vel.x(0);
    cmd.vel.y(0);
    cmd.vel.z(-kVerticalSpeed);
    cmd.acc.setZero();
    cmd.yaw = start_yaw;

    // コマンドを発行
    const auto cmd_ptr = std::make_unique<tobas_msgs::PosVelAccYaw>(cmd);
    cmd_pub_->publish(move(cmd));

    // アクション中止の場合は目標速度・加速度を0にして終了
    if (goal_handle->is_canceling())
    {
      cmd.vel.setZero();
      cmd.acc.setZero();
      cmd_pub_->publish(cmd);

      result->message.clear();
      goal_handle->canceled(result);
      break;
    }

    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }

  is_action_running_ = false;
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandActionServerNode)
