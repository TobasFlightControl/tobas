#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_math/linalg.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_std_tools/gnss.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_trajectory_generators/cubic.hpp>
#include <tobas_trajectory_generators/linear.hpp>
#include <tobas_trajectory_generators/time_optimal.hpp>

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel.hpp>
#include <tobas_command_msgs_adapter/pos_vel_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>
#include <tobas_mission_msgs/action/move.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

#include "tobas_multicopter_actions/common.hpp"

class MoveServerNode : public tobas::BaseNode
{
  using self = MoveServerNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_mission_msgs::action::Move;

  static constexpr double kAttitudeRate = M_PI / 6;  // [rad/s]
  static constexpr double kHeadingRate = M_PI / 3;   // [rad/s]

public:
  explicit MoveServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr gnss_origin_;

  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVel> pos_vel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelYaw> pos_vel_yaw_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelPitchYaw> pos_vel_pitch_yaw_pub_;

  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::GeodeticCoordinates> gnss_origin_sub_;

  ros2::ActionServerPtr<ActionType> as_;

  kdl::Vector computeGoalPosition(const ActionType::Goal::ConstSharedPtr& goal);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

MoveServerNode::MoveServerNode(const rclcpp::NodeOptions& options) : super("move_server", options)
{
  angle_pub_ = createPublisher<tobas_command_msgs::Angle>(tobas::kAngleCmdTopic);
  pos_vel_pub_ = createPublisher<tobas_command_msgs::PosVel>(tobas::kPosVelCmdTopic);
  pos_vel_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelYaw>(tobas::kPosVelYawCmdTopic);
  pos_vel_pitch_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelPitchYaw>(tobas::kPosVelPitchYawCmdTopic);

  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  gnss_origin_sub_ = createSubscriber(tobas::kGnssOriginTopic, &self::gnssOriginCb, this, true, true);

  as_ = createAction(tobas::kMoveAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

kdl::Vector MoveServerNode::computeGoalPosition(const ActionType::Goal::ConstSharedPtr& goal)
{
  kdl::Vector goal_pos;

  const auto& tar_lat = goal->target_latitude;
  const auto& tar_lon = goal->target_longitude;
  const auto& lat_0 = gnss_origin_->latitude;
  const auto& lon_0 = gnss_origin_->longitude;
  tbs::gnssToCartRelative(tar_lat, tar_lon, lat_0, lon_0, goal_pos.x(), goal_pos.y());

  // Z軸
  // TODO: 目標高度がMSLで与えられた場合にも対応
  goal_pos.z(goal->target_altitude);

  return goal_pos;
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
  TOBAS_INFO("Move action is requested.");

  if (goal->target_latitude < -90 || 90 < goal->target_latitude) {
    TOBAS_ERROR("Invalid target latitude.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->target_longitude < -180 || 180 < goal->target_longitude) {
    TOBAS_ERROR("Invalid target longitude.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->max_horizontal_velocity <= 0.) {
    TOBAS_ERROR("Maximum horizontal velocity must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->max_vertical_velocity <= 0.) {
    TOBAS_ERROR("Maximum vertical velocity must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->max_horizontal_accel <= 0.) {
    TOBAS_ERROR("Maximum horizontal acceleration must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->max_vertical_accel <= 0.) {
    TOBAS_ERROR("Maximum vertical acceleration must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->max_horizontal_jerk <= 0.) {
    TOBAS_ERROR("Maximum horizontal jerk must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->max_vertical_jerk <= 0.) {
    TOBAS_ERROR("Maximum vertical jerk must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->acceptance_radius <= 0.) {
    TOBAS_WARN("The acceptance radius is not specified. It will be infinite.");
  }

  if (goal->altitude_tolerance <= 0.) {
    TOBAS_WARN("The altitude tolerance is not specified. It will be infinite.");
  }

  if (goal->timeout <= 0.) {
    TOBAS_WARN("The timeout is not specified. It will be infinite.");
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveServerNode::handleCancel(ros2::ActionGoalHandlePtr<ActionType>)
{
  TOBAS_INFO("Move action is canceled.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveServerNode::execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle)
{
  // Create result
  const auto result = std::make_shared<ActionType::Result>();

  // Check topics
  if (!odom_) {
    result->message = "Odometry is not received yet.";
    goal_handle->abort(result);
    return;
  }
  if (!arming_) {
    result->message = "Arming status is not received yet.";
    goal_handle->abort(result);
    return;
  }
  if (!gnss_origin_) {
    result->message = "GNSS origin is not received yet.";
    goal_handle->abort(result);
    return;
  }

  // Check if rotors are armed
  if (!arming_->data) {
    result->message = "Rotors are not armed.";
    goal_handle->abort(result);
    return;
  }

  // Check odometry
  if (odom_->status != tobas_msgs::msg::Odometry::NO_ERROR) {
    result->message = "There is a problem with the state estimation.";
    goal_handle->abort(result);
    return;
  }

  // 初期状態を取得
  const auto start_time = now();
  const auto start_pos = odom_->frame.p.clone();
  const kdl::Euler start_rpy(odom_->frame.M);
  TOBAS_INFO("Start position: ", start_pos);

  // Get goal
  const auto goal = goal_handle->get_goal();

  // 目標位置を計算
  const auto goal_pos = computeGoalPosition(goal);
  TOBAS_INFO("Goal position: ", goal_pos);

  // 軌道を生成
  const Eigen::Vector2d start_xy(start_pos.x(), start_pos.y());
  const Eigen::Vector2d goal_xy(goal_pos.x(), goal_pos.y());
  const Eigen::Vector2d xy_diff = goal_xy - start_xy;  // XYの軌道を別々に生成すると最短経路を通らないことに注意
  const auto xy_dist = xy_diff.norm();  // [m]
  if (xy_dist == 0.) {
    result->message.clear();
    goal_handle->succeed(result);
    return;
  }
  const Eigen::Vector2d xy_dir = xy_diff / xy_dist;
  const traj::TimeOptimalTrajectory traj_xy(
    0., xy_dist, goal->max_horizontal_jerk, goal->max_horizontal_accel, goal->max_horizontal_velocity);

  const traj::TimeOptimalTrajectory traj_z(
    start_pos.z(), goal_pos.z(), goal->max_vertical_jerk, goal->max_vertical_accel, goal->max_vertical_velocity);

  const auto roll_duration = std::abs(start_rpy.roll) / kAttitudeRate;
  const traj::LinearSpline traj_roll(start_rpy.roll, 0., roll_duration);

  const auto pitch_duration = std::abs(start_rpy.pitch) / kAttitudeRate;
  const traj::LinearSpline traj_pitch(start_rpy.pitch, 0., pitch_duration);

  const auto goal_yaw = atan2(xy_dir.y(), xy_dir.x());
  const auto yaw_diff = algo::wrapPi(goal_yaw - start_rpy.yaw);  // 最短経路をとるよう[-π, π)の範囲に変換
  const auto yaw_duration = std::abs(start_rpy.yaw) / kHeadingRate;
  const traj::CubicSpline traj_yaw(0., yaw_diff, yaw_duration);

  // 所要時間を取得
  const auto duration = std::max(traj_xy.duration(), traj_z.duration());
  TOBAS_INFO("Moving to the target position will take ", duration, " seconds.");

  // 軌道を発行
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // 開始からの経過時間を計算
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();

    // タイムアウトの確認
    if (goal->timeout > 0. && t > duration + goal->timeout) {
      result->message = "Timeout before reaching the goal position.";
      goal_handle->abort(result);
      return;
    }

    // 現在の位置を取得
    const auto& cur_pos = odom_->frame.p;

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    if (t > duration) {
      const auto pos_err = goal_pos - cur_pos;
      const auto xy_err_abs = math::norm(pos_err.x(), pos_err.y());
      const auto z_err_abs = std::abs(pos_err.z());
      const auto hor_ok = goal->acceptance_radius <= 0. || xy_err_abs < goal->acceptance_radius;
      const auto ver_ok = goal->altitude_tolerance <= 0. || z_err_abs < goal->altitude_tolerance;
      if (hor_ok && ver_ok) {
        result->message.clear();
        goal_handle->succeed(result);
        return;
      }
    }

    // 現在の時刻における目標状態を取得
    const auto traj_point_xy = traj_xy.get(t);
    const Eigen::Vector2d pxy = start_xy + traj_point_xy.p * xy_dir;
    const Eigen::Vector2d vxy = traj_point_xy.v * xy_dir;
    const auto traj_point_z = traj_z.get(t);
    const kdl::Vector tar_pos(pxy.x(), pxy.y(), traj_point_z.p);
    kdl::Vector tar_vel(vxy.x(), vxy.y(), traj_point_z.v);
    const auto tar_roll = traj_roll.get(t).p;
    const auto tar_pitch = traj_pitch.get(t).p;
    const auto tar_yaw = algo::wrapPi(start_rpy.yaw + traj_yaw.get(t).p);

    // アクション中止の場合は目標速度を0にする
    if (goal_handle->is_canceling()) {
      tar_vel.setZero();
    }

    // コマンドを発行
    {
      auto cmd = std::make_unique<tobas_command_msgs::Angle>();
      cmd->header.stamp = cur_time;
      cmd->level = goal->level;
      cmd->angle.roll = tar_roll;
      cmd->angle.pitch = tar_pitch;
      cmd->angle.yaw = tar_yaw;
      angle_pub_->publish(std::move(cmd));
    }
    {
      auto cmd = std::make_unique<tobas_command_msgs::PosVel>();
      cmd->header.stamp = cur_time;
      cmd->level = goal->level;
      cmd->pos = tar_pos;
      cmd->vel = tar_vel;
      pos_vel_pub_->publish(std::move(cmd));
    }
    {
      auto cmd = std::make_unique<tobas_command_msgs::PosVelYaw>();
      cmd->header.stamp = cur_time;
      cmd->level = goal->level;
      cmd->pos = tar_pos;
      cmd->vel = tar_vel;
      cmd->yaw = tar_yaw;
      pos_vel_yaw_pub_->publish(std::move(cmd));
    }
    {
      auto cmd = std::make_unique<tobas_command_msgs::PosVelPitchYaw>();
      cmd->header.stamp = cur_time;
      cmd->level = goal->level;
      cmd->pos = tar_pos;
      cmd->vel = tar_vel;
      cmd->pitch = tar_pitch;
      cmd->yaw = tar_yaw;
      pos_vel_pitch_yaw_pub_->publish(std::move(cmd));
    }

    // フィードバックを発行
    auto feedback = std::make_unique<ActionType::Feedback>();
    kdl::vectorKDLToMsg(cur_pos, feedback->current_position);
    kdl::vectorKDLToMsg(tar_pos, feedback->target_position);
    kdl::vectorKDLToMsg(tar_pos - cur_pos, feedback->position_error);
    goal_handle->publish_feedback(std::move(feedback));

    // アクション中止の場合は終了
    if (goal_handle->is_canceling()) {
      assert(tar_vel.squaredNorm() == 0.);
      result->message.clear();
      goal_handle->canceled(result);
      return;
    }

    rate.sleep();
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(MoveServerNode)
