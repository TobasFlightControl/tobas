#include <tobas_constants/constants.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/timestamped_buffer.hpp>
#include <tobas_tools/util.hpp>
#include <tobas_trajectory_generators/linear.hpp>

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel.hpp>
#include <tobas_command_msgs_adapter/pos_vel_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>
#include <tobas_mission_msgs/action/land.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

#include "tobas_multicopter_actions/common.hpp"

class LandServerNode : public tobas::BaseNode
{
  using self = LandServerNode;
  using super = tobas::BaseNode;
  using ActionType = tobas_mission_msgs::action::Land;

  static constexpr double kAttitudeRecoveryRate = M_PI / 6;  // [rad/s]

public:
  explicit LandServerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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

  bool disarmRotors();

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, ActionType::Goal::ConstSharedPtr goal);
  rclcpp_action::CancelResponse handleCancel(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
  void execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle);
};

LandServerNode::LandServerNode(const rclcpp::NodeOptions& options) : super("land_server", options)
{
  angle_pub_ = createPublisher<tobas_command_msgs::Angle>(tobas::kAngleCmdTopic);
  pos_vel_pub_ = createPublisher<tobas_command_msgs::PosVel>(tobas::kPosVelCmdTopic);
  pos_vel_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelYaw>(tobas::kPosVelYawCmdTopic);
  pos_vel_pitch_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelPitchYaw>(tobas::kPosVelPitchYawCmdTopic);

  odom_sub_ = createSubscriber(tobas::addThrotNS(tobas::kOdometryTopic), &self::odomCb, this);
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);

  as_ = createAction(tobas::kLandAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

bool LandServerNode::disarmRotors()
{
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = false;

  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);
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

void LandServerNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}

rclcpp_action::GoalResponse
LandServerNode::handleGoal(const rclcpp_action::GoalUUID&, ActionType::Goal::ConstSharedPtr goal)
{
  TOBAS_INFO("Land action is requested.");

  if (goal->speed <= 0.) {
    TOBAS_ERROR("Descending speed must be positive.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LandServerNode::handleCancel(ros2::ActionGoalHandlePtr<ActionType>)
{
  TOBAS_INFO("Land action is canceled.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void LandServerNode::execute(ros2::ActionGoalHandlePtr<ActionType> goal_handle)
{
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

  // 初期状態を取得
  const auto start_time = now();
  const auto start_pos = odom_->frame.p.clone();
  const kdl::Euler start_rpy(odom_->frame.M);

  // Get goal
  const auto goal = goal_handle->get_goal();

  // 起動を生成
  const auto roll_duration = fabs(start_rpy.roll) / kAttitudeRecoveryRate;
  const auto pitch_duration = fabs(start_rpy.pitch) / kAttitudeRecoveryRate;
  const traj::LinearSpline traj_roll(start_rpy.roll, 0., roll_duration);
  const traj::LinearSpline traj_pitch(start_rpy.pitch, 0., pitch_duration);

  // 姿勢を戻しながら下降
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

    // 現在時刻における目標位置姿勢を計算
    const auto cur_time = now();
    const auto dt = (cur_time - start_time).seconds();
    const kdl::Vector tar_pos(start_pos.x(), start_pos.y(), start_pos.z() - goal->speed * dt);
    kdl::Vector tar_vel(0., 0., -goal->speed);
    const auto tar_roll = traj_roll.get(dt).p;
    const auto tar_pitch = traj_pitch.get(dt).p;
    const auto& tar_yaw = start_rpy.yaw;

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

RCLCPP_COMPONENTS_REGISTER_NODE(LandServerNode)
