#include <tobas_std_tools/trajectory.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_mission_msgs/action/takeoff.hpp>

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
  using ActionType = tobas_mission_msgs::action::Takeoff;

public:
  explicit TakeoffServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  CommandType cmd_;
  double dummy_;

  ros2::PublisherPtr<CommandType> cmd_pub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::ActionServerPtr<ActionType> as_;

  bool armRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

TakeoffServerNode::TakeoffServerNode(const rclcpp::NodeOptions& options) : super("takeoff_server", options)
{
  cmd_pub_ = createPublisher<CommandType>(tobas::kPosVelYawCmdTopic);
  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  as_ = createAction(tobas::kTakeoffAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool TakeoffServerNode::armRotors()
{
  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = true;
  if (!sc.call(req))
  {
    TOBAS_ERROR("Failed to call \"", tobas::kSetArmSrv, "\" service.");
    return false;
  }

  const auto res = sc.getResponse();
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

  // Check topics
  if (odom_ == nullptr)
  {
    result->message = "Odometry is not received yet.";
    goal_handle->abort(result);
    return;
  }

  // Arm rotors
  if (!armRotors())
    return;

  // TODO: 正常にアームされたかどうかを確認

  // Get goal
  const auto goal = goal_handle->get_goal();

  // 軌道を生成
  tobas_std::CubicSpline traj_z(odom_->frame.p.z(), goal->target_altitude, goal->duration);
  const auto duration = traj_z.duration();

  // 初期状態
  const auto start_time = get_clock()->now();
  const auto start_x = odom_->frame.p.x();
  const auto start_y = odom_->frame.p.y();
  const auto start_yaw = odom_->frame.M.getYaw();

  // 軌道を発行
  rclcpp::Rate rate(kCommandRate, get_clock());
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
    const auto alt_error = fabs(goal->target_altitude - odom_->frame.p.z());
    if (t > duration && alt_error < goal->altitude_tolerance)
    {
      result->message.clear();
      goal_handle->succeed(result);
      return;
    }

    // コマンドを作成
    cmd_.level = goal->level;
    cmd_.pos.setZero();
    cmd_.vel.setZero();

    // 水平位置とヨー角は初期状態を維持
    cmd_.pos.x(start_x);
    cmd_.pos.y(start_y);
    cmd_.yaw = start_yaw;

    // 鉛直方向の軌道を生成
    traj_z.get(t, cmd_.pos.z(), cmd_.vel.z(), dummy_);

    // コマンドを発行
    auto cmd_ptr = std::make_unique<CommandType>(cmd_);
    cmd_pub_->publish(move(cmd_ptr));

    // アクション中止の場合は目標速度を0にして終了
    if (goal_handle->is_canceling())
    {
      cmd_.vel.setZero();
      cmd_pub_->publish(cmd_);

      result->message.clear();
      goal_handle->canceled(result);
      return;
    }

    rate.sleep();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(TakeoffServerNode)
