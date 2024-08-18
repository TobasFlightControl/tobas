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

public:
  explicit LandActionServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_std::TimestampedBuffer<double> alt_buf_;
  tobas_msgs::PosVelAccYaw cmd_;

  PublisherPtr<tobas_msgs::PosVelAccYaw> cmd_pub_;
  ActionPtr<ActionType> as_;

  bool disarmRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ActionGoalHandlePtr<ActionType> goal_handle);
};

LandActionServerNode::LandActionServerNode(const rclcpp::NodeOptions& options)
  : super("land_action_server", options), alt_buf_(kTimeWindow)
{
  cmd_pub_ = createPublisher<tobas_msgs::PosVelAccYaw>(tobas::kPosVelAccYawCmdTopic);
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

rclcpp_action::CancelResponse LandActionServerNode::handleCancel(ActionGoalHandlePtr<ActionType>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LandActionServerNode::execute(ActionGoalHandlePtr<ActionType> goal_handle)
{
  TOBAS_INFO("Landing action is executing.");

  const auto result = std::make_shared<ActionType::Result>();

  // ベース状態のデータを初期化
  odom_ = nullptr;
  alt_buf_.clear();

  // 一時的にオドメトリの購読を開始
  const auto odom_sub = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);

  // オドメトリが発行されていることを確認
  rclcpp::spin_some(shared_from_this());
  if (odom_ == nullptr)
  {
    result->message = "Odometry is not received yet.";
    goal_handle->abort(result);
  }

  // 初期状態
  const auto start_time = get_clock()->now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_z = odom_->frame.p.z();
  const auto start_yaw = kdl::Euler(odom_->frame.M).yaw;

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
    const auto cmd_ptr = std::make_unique<tobas_msgs::PosVelAccYaw>(cmd_);
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

    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandActionServerNode)
