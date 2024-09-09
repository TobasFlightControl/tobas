#include <tobas_std_tools/trajectory.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_ros2_tools/simple_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/Odometry.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs/action/takeoff.hpp>

#include "../include/tobas_mr_actions/common.hpp"

using namespace std;

/**
 * @brief マルチコプターの離陸指令を発行するアクションサーバ．
 * X,Y,Yawをアクション開始時の値に保ったままZのみを増やしていく．
 * cf. https://docs.px4.io/main/en/flight_modes/takeoff.html
 */
class TakeoffServerNode : public tobas::BaseNode
{
  using self = TakeoffServerNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_msgs::action::Takeoff;

public:
  explicit TakeoffServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  CommandType cmd_;

  ros2::PublisherPtr<CommandType> cmd_pub_;
  ros2::ActionPtr<ActionType> as_;

  bool armRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

TakeoffServerNode::TakeoffServerNode(const rclcpp::NodeOptions& options) : super("mr_takeoff_action_server", options)
{
  cmd_pub_ = createPublisher<CommandType>(tobas::kPosVelAccYawCmdTopic);
  as_ = createAction(tobas::kLandAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool TakeoffServerNode::armRotors()
{
  ros2::SimpleServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = true;
  if (!sc.call(req))
  {
    TOBAS_ERROR("Failed to call \"", tobas::kSetArmSrv, "\" service.");
    return false;
  }

  const auto& res = sc.getResponse();
  if (!res->success)
  {
    TOBAS_ERROR("Failed to arm rotors: ", res->message);
    return false;
  }

  return true;
}

void TakeoffServerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

rclcpp_action::GoalResponse
TakeoffServerNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr goal)
{
  if (goal->target_altitude <= 0)
  {
    TOBAS_ERROR("Target altitude must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->altitude_tolerance <= 0)
  {
    TOBAS_ERROR("Altitude tolerance must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->duration <= 0)
  {
    TOBAS_ERROR("Target duration must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TakeoffServerNode::handleCancel(ros2::ActionGoalHandlePtr<ActionType>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TakeoffServerNode::execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle)
{
  TOBAS_INFO("Takeoff action is requested.");

  // Create result
  const auto result = std::make_shared<ActionType::Result>();

  // 一時的にトピックを購読
  const auto odom_sub = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);

  // オドメトリを受け取るまで待機
  TOBAS_INFO("Waiting for odometry.");
  rclcpp::Rate wait_for_topic_rate(kWaitForTopicRate);
  while (rclcpp::ok())
  {
    if (odom_ != nullptr)
      break;

    if (goal_handle->is_canceling())
    {
      result->message = "Failed to get odometry.";
      goal_handle->canceled(result);
      return;
    }

    wait_for_topic_rate.sleep();
  }

  // Check odometry
  if (odom_->status != tobas_msgs::msg::Odometry::NO_ERROR)
  {
    result->message = "There is a problem with the state estimation.";
    goal_handle->abort(result);
    return;
  }

  // Arm rotors
  if (!armRotors())
    return;

  // Get goal
  const auto goal = goal_handle->get_goal();

  // 軌道を生成
  tobas_std::CubicSpline traj_z(odom_->frame.p.z(), goal->target_altitude, goal->duration);
  const auto duration = traj_z.duration();

  // 初期状態
  const auto start_time = get_clock()->now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_yaw = kdl::Euler(odom_->frame.M).yaw;

  // 軌道を発行
  rclcpp::Rate cmd_rate(kCommandRate);
  while (rclcpp::ok())
  {
    // 開始からの経過時間を計算
    const auto t = (get_clock()->now() - start_time).seconds();

    // タイムアウトの確認
    if (goal->timeout > 0 && t > duration + goal->timeout)
    {
      result->message = "Timeout before reaching the target altitude.";
      goal_handle->abort(result);
      return;
    }

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    const auto alt_error = abs(goal->target_altitude - odom_->frame.p.z());
    if (t > duration && alt_error < goal->altitude_tolerance)
    {
      result->message.clear();
      goal_handle->succeed(result);
      return;
    }

    // コマンドを作成
    cmd_.level = goal->level;
    cmd_.frame_id.data = tobas_msgs::msg::FrameId::WORLD;
    cmd_.pos.setZero();
    cmd_.vel.setZero();
    cmd_.acc.setZero();

    // 水平位置とヨー角は初期状態を維持
    cmd_.pos.x(start_x);
    cmd_.pos.y(start_y);
    cmd_.yaw = start_yaw;

    // 鉛直方向の軌道を生成
    traj_z.get(t, cmd_.pos.z(), cmd_.vel.z(), cmd_.acc.z());

    // コマンドを発行
    auto cmd_ptr = std::make_unique<CommandType>(cmd_);
    cmd_pub_->publish(move(cmd_ptr));

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

    cmd_rate.sleep();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeoffServerNode)
