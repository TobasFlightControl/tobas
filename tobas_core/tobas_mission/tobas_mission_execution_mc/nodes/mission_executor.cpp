#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_kdl_conversions/kdl_msg.hpp>
#include <tobas_math/linalg.hpp>
#include <tobas_mission_items/mission_items.hpp>
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
#include <tobas_mission_msgs/action/execute_mission.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

namespace tobas
{
namespace mission
{
class MulticopterMissionExecutorNode : public BaseNode
{
  using self = MulticopterMissionExecutorNode;
  using super = BaseNode;

  using ActionType = tobas_mission_msgs::action::ExecuteMission;
  using GoalHandlePtr = ros2::ActionGoalHandlePtr<ActionType>;
  using GoalPtr = ActionType::Goal::ConstSharedPtr;
  using ResultPtr = ActionType::Result::SharedPtr;

  static constexpr double kCommandRate = 100.;       // [Hz]
  static constexpr double kAttitudeRate = M_PI / 6;  // [rad/s]
  static constexpr double kHeadingRate = M_PI / 3;   // [rad/s]

public:
  explicit MulticopterMissionExecutorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr gnss_origin_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;

  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVel> pos_vel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelYaw> pos_vel_yaw_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelPitchYaw> pos_vel_pitch_yaw_pub_;

  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::GeodeticCoordinates> gnss_origin_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;

  ros2::ActionServerPtr<ActionType> as_;

  void publishCommands(
    const rclcpp::Time& stamp,
    const kdl::Vector& pos,
    const kdl::Vector& vel,
    double roll,
    double pitch,
    double yaw) const;
  bool armRotors(bool arming);

  bool executeWaypoint(const Waypoint& goal, GoalHandlePtr goal_handle, ResultPtr result);
  bool executeTakeoff(const Takeoff& goal, GoalHandlePtr goal_handle, ResultPtr result);
  bool executeLand(const Land& goal, GoalHandlePtr goal_handle, ResultPtr result);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, GoalPtr goal);
  rclcpp_action::CancelResponse handleCancel(GoalHandlePtr goal_handle);
  void execute(GoalHandlePtr goal_handle);
};

MulticopterMissionExecutorNode::MulticopterMissionExecutorNode(const rclcpp::NodeOptions& options)
  : super("mission_executor", options)
{
  angle_pub_ = createPublisher<tobas_command_msgs::Angle>(kAngleCmdTopic);
  pos_vel_pub_ = createPublisher<tobas_command_msgs::PosVel>(kPosVelCmdTopic);
  pos_vel_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelYaw>(kPosVelYawCmdTopic);
  pos_vel_pitch_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelPitchYaw>(kPosVelPitchYawCmdTopic);

  odom_sub_ = createSubscriber(addThrotNS(kOdometryTopic), &self::odomCb, this);
  arming_sub_ = createSubscriber(kArmingTopic, &self::armingCb, this);
  gnss_origin_sub_ = createSubscriber(kGnssOriginTopic, &self::gnssOriginCb, this, true, true);
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);

  as_ = createAction(kMoveAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

void MulticopterMissionExecutorNode::publishCommands(
  const rclcpp::Time& stamp,
  const kdl::Vector& pos,
  const kdl::Vector& vel,
  double roll,
  double pitch,
  double yaw) const
{
  {
    auto cmd = std::make_unique<tobas_command_msgs::Angle>();
    cmd->header.stamp = stamp;
    cmd->angle.roll = roll;
    cmd->angle.pitch = pitch;
    cmd->angle.yaw = yaw;
    angle_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVel>();
    cmd->header.stamp = stamp;
    cmd->pos = pos;
    cmd->vel = vel;
    pos_vel_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVelYaw>();
    cmd->header.stamp = stamp;
    cmd->pos = pos;
    cmd->vel = vel;
    cmd->yaw = yaw;
    pos_vel_yaw_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVelPitchYaw>();
    cmd->header.stamp = stamp;
    cmd->pos = pos;
    cmd->vel = vel;
    cmd->pitch = pitch;
    cmd->yaw = yaw;
    pos_vel_pitch_yaw_pub_->publish(std::move(cmd));
  }
}

bool MulticopterMissionExecutorNode::armRotors(bool arming)
{
  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), tobas::kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  if (!sc.call(req)) {
    TOBAS_ERROR("Failed to call \"", tobas::kSetArmSrv, "\" service.");
    return false;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    TOBAS_ERROR("Failed to set the arming status: ", res->message);
    return false;
  }

  return true;
}

bool MulticopterMissionExecutorNode::executeWaypoint(const Waypoint& goal, GoalHandlePtr goal_handle, ResultPtr result)
{
  // Check if rotors are armed
  if (!arming_->data) {
    result->message = "Rotors are disarmed.";
    goal_handle->abort(result);
    return false;
  }

  // 初期状態を取得
  const auto start_time = now();
  const auto start_pos = odom_->frame.p.clone();
  const kdl::Euler start_rpy(odom_->frame.M);
  TOBAS_INFO("Start position: ", start_pos);

  // 目標位置を計算
  kdl::Vector goal_pos;
  tbs::gnssToCartRelative(
    goal.latitude, goal.longitude, gnss_origin_->latitude, gnss_origin_->longitude, goal_pos.x(), goal_pos.y());
  goal_pos.z(goal.altitude);  // TODO: 目標高度がMSLで与えられた場合にも対応
  TOBAS_INFO("Goal position: ", goal_pos);

  // 軌道を生成
  const Eigen::Vector2d start_xy(start_pos.x(), start_pos.y());
  const Eigen::Vector2d goal_xy(goal_pos.x(), goal_pos.y());
  const Eigen::Vector2d xy_diff = goal_xy - start_xy;  // XYの軌道を別々に生成すると最短経路を通らないことに注意
  const auto xy_dist = xy_diff.norm();  // [m]
  if (xy_dist == 0.) {
    return true;
  }
  const Eigen::Vector2d xy_dir = xy_diff / xy_dist;
  const traj::TimeOptimalTrajectory traj_xy(
    0., xy_dist, goal.max_horizontal_jerk, goal.max_horizontal_accel, goal.max_horizontal_velocity);

  const traj::TimeOptimalTrajectory traj_z(
    start_pos.z(), goal_pos.z(), goal.max_vertical_jerk, goal.max_vertical_accel, goal.max_vertical_velocity);

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
    if (goal.timeout > 0. && t > duration + goal.timeout) {
      result->message = "Timeout before reaching the goal position.";
      goal_handle->abort(result);
      return false;
    }

    // 現在の位置を取得
    const auto& cur_pos = odom_->frame.p;

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    if (t > duration) {
      const auto pos_err = goal_pos - cur_pos;
      const auto xy_err_abs = math::norm(pos_err.x(), pos_err.y());
      const auto z_err_abs = std::abs(pos_err.z());
      const auto hor_ok = goal.acceptance_radius <= 0. || xy_err_abs < goal.acceptance_radius;
      const auto ver_ok = goal.altitude_tolerance <= 0. || z_err_abs < goal.altitude_tolerance;
      if (hor_ok && ver_ok) {
        return true;
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
    publishCommands(cur_time, tar_pos, tar_vel, tar_roll, tar_pitch, tar_yaw);

    // アクション中止の場合は終了
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return false;
    }

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeTakeoff(const Takeoff& goal, GoalHandlePtr goal_handle, ResultPtr result)
{
  // Arm rotors
  if (!armRotors(true)) {
    result->message = "Failed to arm rotors.";
    goal_handle->abort(result);
    return false;
  }

  // TODO: 正常にアームされたかどうかを確認

  // 初期状態を取得
  const auto start_time = now();
  const auto start_pos = odom_->frame.p.clone();
  const auto start_yaw = odom_->frame.M.getYaw();

  // 軌道を生成
  const traj::TimeOptimalTrajectory traj_z(start_pos.z(), goal.altitude, goal.max_jerk, goal.max_accel, goal.max_speed);

  // 所要時間を取得
  const auto duration = traj_z.duration();
  TOBAS_INFO("Takeoff will take ", duration, " seconds.");

  // 目標状態の固定部分を作成
  kdl::Vector tar_pos(start_pos.x(), start_pos.y(), NAN);
  kdl::Vector tar_vel(0., 0., NAN);

  // 軌道を発行
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // 開始からの経過時間を計算
    const auto cur_time = now();
    const auto dt = (cur_time - start_time).seconds();

    // タイムアウトの確認
    if (goal.timeout > 0. && dt > duration + goal.timeout) {
      result->message = "Timeout before reaching the target altitude.";
      goal_handle->abort(result);
      return false;
    }

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    const auto& cur_pos = odom_->frame.p;
    const auto alt_err_abs = std::abs(goal.altitude - cur_pos.z());
    if (dt > duration) {
      if (goal.altitude_tolerance <= 0. || alt_err_abs < goal.altitude_tolerance) {
        return true;
      }
    }

    // 鉛直方向の軌道を生成
    const auto traj_point_z = traj_z.get(dt);
    tar_pos.z(traj_point_z.p);
    tar_vel.z(traj_point_z.v);

    // アクション中止の場合は目標速度を0にする
    if (goal_handle->is_canceling()) {
      tar_vel.setZero();
    }

    // コマンドを発行
    publishCommands(cur_time, tar_pos, tar_vel, 0., 0., start_yaw);

    // アクション中止の場合は終了
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return false;
    }

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeLand(const Land& goal, GoalHandlePtr goal_handle, ResultPtr result)
{
  // 初期状態を取得
  const auto start_time = now();
  const auto start_pos = odom_->frame.p.clone();
  const kdl::Euler start_rpy(odom_->frame.M);

  // 起動を生成
  const auto roll_duration = std::abs(start_rpy.roll) / kAttitudeRate;
  const auto pitch_duration = std::abs(start_rpy.pitch) / kAttitudeRate;
  const traj::LinearSpline traj_roll(start_rpy.roll, 0., roll_duration);
  const traj::LinearSpline traj_pitch(start_rpy.pitch, 0., pitch_duration);

  // 姿勢を戻しながら下降
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // 着陸検知したらモータを停止して終了
    if (landed_->data) {
      TOBAS_INFO("Landing detected. Stopping motors.");
      if (!armRotors(false)) {
        result->message = "Failed to disarm rotors.";
        goal_handle->abort(result);
        return false;
      }
      return true;
    }

    // 現在時刻における目標位置姿勢を計算
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();
    const auto tar_z = start_pos.z() - goal.speed * t;
    const kdl::Vector tar_pos(start_pos.x(), start_pos.y(), tar_z);
    kdl::Vector tar_vel(0., 0., -goal.speed);
    const auto tar_roll = traj_roll.get(t).p;
    const auto tar_pitch = traj_pitch.get(t).p;
    const auto& tar_yaw = start_rpy.yaw;

    // アクション中止の場合は目標速度を0にする
    if (goal_handle->is_canceling()) {
      tar_vel.setZero();
    }

    // コマンドを発行
    publishCommands(cur_time, tar_pos, tar_vel, tar_roll, tar_pitch, tar_yaw);

    // アクション中止の場合は終了
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return false;
    }

    rate.sleep();
  }

  return false;
}

void MulticopterMissionExecutorNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void MulticopterMissionExecutorNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
}

void MulticopterMissionExecutorNode::gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin)
{
  gnss_origin_ = gnss_origin;
}

void MulticopterMissionExecutorNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}

rclcpp_action::GoalResponse MulticopterMissionExecutorNode::handleGoal(const rclcpp_action::GoalUUID&, GoalPtr goal)
{
  TOBAS_INFO("New mission is uploaded.");

  for (const auto& [idx, item] : std::views::enumerate(goal->mission.items)) {
    switch (item.type) {
      case kWaypoint: {
        Waypoint waypoint;
        memcpy(&waypoint, item.data.data(), sizeof(Waypoint));

        if (waypoint.latitude < -90 || 90 < waypoint.latitude) {
          TOBAS_ERROR("Mission No. ", idx, ": Invalid target latitude.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.longitude < -180 || 180 < waypoint.longitude) {
          TOBAS_ERROR("Mission No. ", idx, ": Invalid target longitude.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.max_horizontal_velocity <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum horizontal velocity must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.max_vertical_velocity <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum vertical velocity must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.max_horizontal_accel <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum horizontal acceleration must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.max_vertical_accel <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum vertical acceleration must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.max_horizontal_jerk <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum horizontal jerk must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.max_vertical_jerk <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum vertical jerk must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.acceptance_radius <= 0.) {
          TOBAS_WARN("Mission No. ", idx, ": The acceptance radius is not specified. It will be infinite.");
        }
        if (waypoint.altitude_tolerance <= 0.) {
          TOBAS_WARN("Mission No. ", idx, ": The altitude tolerance is not specified. It will be infinite.");
        }
        if (waypoint.timeout <= 0.) {
          TOBAS_WARN("Mission No. ", idx, ": The timeout is not specified. It will be infinite.");
        }

        break;
      }
      case kTakeoff: {
        Takeoff takeoff;
        memcpy(&takeoff, item.data.data(), sizeof(Takeoff));

        if (takeoff.altitude <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Target altitude must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (takeoff.max_speed <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum speed must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (takeoff.max_accel <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum acceleration must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (takeoff.max_jerk <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Maximum jerk must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (takeoff.altitude_tolerance <= 0.) {
          TOBAS_WARN("Mission No. ", idx, ": The altitude tolerance is not specified. It will be infinite.");
        }
        if (takeoff.timeout <= 0.) {
          TOBAS_WARN("Mission No. ", idx, ": The timeout is not specified. It will be infinite.");
        }

        break;
      }
      case kLand: {
        Land land;
        memcpy(&land, item.data.data(), sizeof(Land));

        if (land.speed <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Descending speed must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        break;
      }
      default: {
        TOBAS_ERROR("Invalid mission type: ", (int)item.type);
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MulticopterMissionExecutorNode::handleCancel(GoalHandlePtr)
{
  TOBAS_INFO("The current mission is canceled.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MulticopterMissionExecutorNode::execute(GoalHandlePtr goal_handle)
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
  if (!landed_) {
    result->message = "Landed state is not received yet.";
    goal_handle->abort(result);
    return;
  }

  // Get goal
  const auto goal = goal_handle->get_goal();

  // Execute mission
  for (const auto& [idx, item] : std::views::enumerate(goal->mission.items)) {
    // Publish the current mission #
    auto feedback = std::make_unique<ActionType::Feedback>();
    feedback->current_index = idx;
    goal_handle->publish_feedback(std::move(feedback));

    switch (item.type) {
      case kWaypoint: {
        Waypoint waypoint;
        memcpy(&waypoint, item.data.data(), sizeof(Waypoint));
        if (!executeWaypoint(waypoint, goal_handle, result)) {
          return;
        }
        break;
      }
      case kTakeoff: {
        Takeoff takeoff;
        memcpy(&takeoff, item.data.data(), sizeof(Takeoff));
        if (!executeTakeoff(takeoff, goal_handle, result)) {
          return;
        }
        break;
      }
      case kLand: {
        Land land;
        memcpy(&land, item.data.data(), sizeof(Land));
        if (!executeLand(land, goal_handle, result)) {
          return;
        }
        break;
      }
      default: {
        result->message = "Invalid mission type: " + std::to_string(idx);
        goal_handle->abort(result);
        return;
      }
    }
  }

  result->message.clear();
  goal_handle->succeed(result);
}
}  // namespace mission
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::mission::MulticopterMissionExecutorNode)
