#include <tobas_algorithm/core.hpp>
#include <tobas_std_tools/trajectory.hpp>
#include <tobas_std_tools/geometry.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs/action/move.hpp>

#include "../include/tobas_mr_actions/common.hpp"

using namespace std;

class MoveServerNode : public tobas::BaseNode
{
  using self = MoveServerNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_msgs::action::Move;

public:
  explicit MoveServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr gnss_origin_;
  CommandType cmd_;

  ros2::PublisherPtr<CommandType> cmd_pub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::GeodeticCoordinates> gnss_origin_sub_;
  ros2::ActionServerPtr<ActionType> as_;

  bool computeGoalPosition(const ActionType::Goal::ConstSharedPtr& goal, kdl::Vector& goal_pos);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

MoveServerNode::MoveServerNode(const rclcpp::NodeOptions& options) : super("move_server", options)
{
  cmd_pub_ = createPublisher<CommandType>(tobas::kPosVelAccYawCmdTopic);

  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  gnss_origin_sub_ = createSubscriber(tobas::kGnssOriginTopic, &self::gnssOriginCb, this, true, true);

  as_ = createAction(tobas::kMoveAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool MoveServerNode::computeGoalPosition(const ActionType::Goal::ConstSharedPtr& goal, kdl::Vector& goal_pos)
{
  const auto& tar_lat = goal->target_latitude;
  const auto& tar_lon = goal->target_longitude;
  const auto& lat_0 = gnss_origin_->latitude;
  const auto& lon_0 = gnss_origin_->longitude;
  tobas_std::gnssToCartRelative(tar_lat, tar_lon, lat_0, lon_0, goal_pos.x(), goal_pos.y());

  // Z軸
  // TODO: 目標高度がMSLで与えられた場合にも対応
  goal_pos.z(goal->target_altitude);

  return true;
}

void MoveServerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void MoveServerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void MoveServerNode::gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin)
{
  gnss_origin_ = gnss_origin;
}

rclcpp_action::GoalResponse
MoveServerNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr goal)
{
  if (goal->target_latitude < -90 || 90 < goal->target_latitude)
  {
    TOBAS_ERROR("Invalid target latitude.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->target_longitude < -180 || 180 < goal->target_longitude)
  {
    TOBAS_ERROR("Invalid target longitude.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->acceptance_radius <= 0)
  {
    TOBAS_ERROR("Acceptance radius must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->duration <= 0)
  {
    TOBAS_ERROR("Target duration must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveServerNode::handleCancel(ros2::ActionGoalHandlePtr<ActionType>)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveServerNode::execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle)
{
  TOBAS_INFO("Moving action is requested.");

  // Create result
  const auto result = std::make_shared<ActionType::Result>();

  // Check if necessary topics are received
  if (odom_ == nullptr)
  {
    result->message = "Odometry is not received yet.";
    goal_handle->abort(result);
    return;
  }
  if (arming_ == nullptr)
  {
    result->message = "Arming status is not received yet.";
    goal_handle->abort(result);
    return;
  }
  if (gnss_origin_ == nullptr)
  {
    result->message = "GNSS origin is not received yet.";
    goal_handle->abort(result);
    return;
  }

  // Check if rotors are armed
  if (!arming_->data)
  {
    result->message = "Rotors are not armed.";
    goal_handle->abort(result);
    return;
  }

  // Check odometry
  if (odom_->status != tobas_msgs::msg::Odometry::NO_ERROR)
  {
    result->message = "There is a problem with the state estimation.";
    goal_handle->abort(result);
    return;
  }

  // Get goal
  const auto goal = goal_handle->get_goal();

  // 目標位置
  kdl::Vector goal_pos;
  if (!computeGoalPosition(goal, goal_pos))
  {
    result->message = "Failed to compute the goal position.";
    goal_handle->abort(result);
    return;
  }

  // 軌道を生成
  // TODO: 最高速度を考慮して起動を作成
  tobas_std::CubicSpline traj_x(odom_->frame.p.x(), goal_pos.x(), goal->duration);
  tobas_std::CubicSpline traj_y(odom_->frame.p.y(), goal_pos.y(), goal->duration);
  tobas_std::CubicSpline traj_z(odom_->frame.p.z(), goal_pos.z(), goal->duration);
  const auto duration = algo::max(traj_x.duration(), traj_y.duration(), traj_z.duration());

  // 初期状態
  const auto start_time = get_clock()->now();
  const auto start_yaw = kdl::Euler(odom_->frame.M).yaw;

  // 軌道を発行
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok())
  {
    // 開始からの経過時間を計算
    const auto t = (get_clock()->now() - start_time).seconds();

    // タイムアウトの確認
    if (goal->timeout > 0 && t > duration + goal->timeout)
    {
      result->message = "Timeout before reaching the goal position.";
      goal_handle->abort(result);
      return;
    }

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    const auto& cur_pos = odom_->frame.p;
    const auto pos_error = goal_pos - cur_pos;
    if (t > duration && pos_error.norm() < goal->acceptance_radius)
    {
      result->message.clear();
      goal_handle->succeed(result);
      return;
    }

    // コマンドを作成
    cmd_.level = goal->level;
    cmd_.frame_id.data = tobas_msgs::msg::FrameId::WORLD;

    // ヨー角は初期状態を維持
    cmd_.yaw = start_yaw;

    // 現在の時刻における目標状態を取得
    traj_x.get(t, cmd_.pos.x(), cmd_.vel.x(), cmd_.acc.x());
    traj_y.get(t, cmd_.pos.y(), cmd_.vel.y(), cmd_.acc.y());
    traj_z.get(t, cmd_.pos.z(), cmd_.vel.z(), cmd_.acc.z());

    // コマンドを発行
    auto cmd_ptr = std::make_unique<CommandType>(cmd_);
    cmd_pub_->publish(move(cmd_ptr));

    // フィードバックを発行
    auto feedback = std::make_unique<ActionType::Feedback>();
    kdl::vectorKDLToMsg(cur_pos, feedback->current_position);
    kdl::vectorKDLToMsg(cmd_.pos, feedback->target_position);
    kdl::vectorKDLToMsg(cmd_.pos - cur_pos, feedback->position_error);
    goal_handle->publish_feedback(move(feedback));

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

RCLCPP_COMPONENTS_REGISTER_NODE(MoveServerNode)
