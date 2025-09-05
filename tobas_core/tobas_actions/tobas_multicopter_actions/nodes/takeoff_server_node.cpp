#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_trajectory_generators/cubic.hpp>

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel.hpp>
#include <tobas_command_msgs_adapter/pos_vel_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>
#include <tobas_mission_msgs/action/takeoff.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

#include "tobas_multicopter_actions/common.hpp"

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
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;

  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVel> pos_vel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelYaw> pos_vel_yaw_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelPitchYaw> pos_vel_pitch_yaw_pub_;

  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;

  ros2::ActionServerPtr<ActionType> as_;

  bool armRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

TakeoffServerNode::TakeoffServerNode(const rclcpp::NodeOptions& options) : super("takeoff_server", options)
{
  angle_pub_ = createPublisher<tobas_command_msgs::Angle>(tobas::kAngleCmdTopic);
  pos_vel_pub_ = createPublisher<tobas_command_msgs::PosVel>(tobas::kPosVelCmdTopic);
  pos_vel_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelYaw>(tobas::kPosVelYawCmdTopic);
  pos_vel_pitch_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelPitchYaw>(tobas::kPosVelPitchYawCmdTopic);

  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);

  as_ = createAction(tobas::kTakeoffAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool TakeoffServerNode::armRotors()
{
  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = true;
  if (!sc.call(req)) {
    TOBAS_ERROR("Failed to call \"", tobas::kSetArmSrv, "\" service.");
    return false;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    TOBAS_ERROR("Failed to arm rotors: ", res->message);
    return false;
  }

  return true;
}

void TakeoffServerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void TakeoffServerNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}

rclcpp_action::GoalResponse
TakeoffServerNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr goal)
{
  if (goal->target_altitude <= 0.) {
    TOBAS_ERROR("Target altitude must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->altitude_tolerance <= 0.) {
    TOBAS_ERROR("Altitude tolerance must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->duration <= 0.) {
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
  if (!odom_) {
    result->message = "Odometry is not received yet.";
    goal_handle->abort(result);
    return;
  }
  if (!landed_) {
    result->message = "Landed state is not received yet.";
    goal_handle->abort(result);
    return;
  }

  // Check landed state
  if (!landed_->data) {
    result->message = "The aircraft has already taken off.";
    goal_handle->abort(result);
    return;
  }

  // Arm rotors
  if (!armRotors()) {
    return;
  }

  // TODO: 正常にアームされたかどうかを確認

  // 初期状態を取得
  const auto start_time = get_clock()->now();
  const auto start_pos = odom_->frame.p.clone();
  const auto start_yaw = odom_->frame.M.getYaw();

  // Get goal
  const auto goal = goal_handle->get_goal();

  // 軌道を生成
  traj::CubicSpline traj_z(start_pos.z(), goal->target_altitude, goal->duration);

  // 目標状態の固定部分を作成
  kdl::Vector tar_pos(start_pos.x(), start_pos.y(), NAN);
  kdl::Vector tar_vel(0., 0., NAN);

  // 軌道を発行
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // 開始からの経過時間を計算
    const auto cur_time = get_clock()->now();
    const auto dt = (cur_time - start_time).seconds();

    // タイムアウトの確認
    if (goal->timeout > 0. && dt > goal->duration + goal->timeout) {
      result->message = "Timeout before reaching the target altitude.";
      goal_handle->abort(result);
      return;
    }

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    const auto& cur_pos = odom_->frame.p;
    const auto alt_error = fabs(goal->target_altitude - cur_pos.z());
    if (dt > goal->duration && alt_error < goal->altitude_tolerance) {
      result->message.clear();
      goal_handle->succeed(result);
      return;
    }

    // 鉛直方向の軌道を生成
    traj_z.get(dt, tar_pos.z(), tar_vel.z());

    // アクション中止の場合は目標速度を0にする
    if (goal_handle->is_canceling()) {
      tar_vel.setZero();
    }

    // コマンドを発行
    {
      auto cmd = std::make_unique<tobas_command_msgs::Angle>();
      cmd->header.stamp = cur_time;
      cmd->level = goal->level;
      cmd->angle.roll = 0.;
      cmd->angle.pitch = 0.;
      cmd->angle.yaw = start_yaw;
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
      cmd->yaw = start_yaw;
      pos_vel_yaw_pub_->publish(std::move(cmd));
    }
    {
      auto cmd = std::make_unique<tobas_command_msgs::PosVelPitchYaw>();
      cmd->header.stamp = cur_time;
      cmd->level = goal->level;
      cmd->pos = tar_pos;
      cmd->vel = tar_vel;
      cmd->pitch = 0.;
      cmd->yaw = start_yaw;
      pos_vel_pitch_yaw_pub_->publish(std::move(cmd));
    }

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

RCLCPP_COMPONENTS_REGISTER_NODE(TakeoffServerNode)
