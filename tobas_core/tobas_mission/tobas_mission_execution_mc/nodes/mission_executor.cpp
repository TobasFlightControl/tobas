// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_math/linalg.hpp>
#include <tobas_mission_items/mission_items.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_std_tools/gnss.hpp>
#include <tobas_trajectory_generation/offline/linear.hpp>
#include <tobas_trajectory_generation/offline/time_optimal.hpp>

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>
#include <tobas_mission_msgs/action/execute_mission.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/srv/set_arm.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

#include "tobas_mission_execution_mc/stop_trajectory_generator.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace mission
{
class MulticopterMissionExecutorNode : public BaseNode
{
  using self = MulticopterMissionExecutorNode;
  using super = BaseNode;

  using Action = tobas_mission_msgs::action::ExecuteMission;
  using Goal = Action::Goal;
  using GoalPtr = Goal::ConstSharedPtr;
  using Result = Action::Result;
  using ResultPtr = Result::SharedPtr;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using GoalHandlePtr = std::shared_ptr<GoalHandle>;

  static constexpr double kCommandRate = 100.;       // [Hz]
  static constexpr double kAttitudeRate = M_PI / 6;  // [rad/s]
  static constexpr double kMinBrakeDuration = 0.1;   // [s]

public:
  explicit MulticopterMissionExecutorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  struct WaypointConfig
  {
    double max_hor_vel;    // [m/s]
    double max_hor_acc;    // [m/s^2]
    double max_hor_jerk;   // [m/s^3]
    double max_ver_vel;    // [m/s]
    double max_ver_acc;    // [m/s^2]
    double max_ver_jerk;   // [m/s^3]
    double max_head_rate;  // [rad/s]
    double max_head_acc;   // [rad/s^2]
  } wp_cfg_;
  struct TakeoffConfig
  {
    double max_speed;  // [m/s]
    double max_accel;  // [m/s^2]
    double max_jerk;   // [m/s^3]
  } takeoff_cfg_;
  struct LandConfig
  {
    double speed;  // [m/s]
  } land_cfg_;
  struct ReturnToLaunchConfig
  {
    double min_alt;  // [m]
  } rtl_cfg_;

  bool is_executing_ = false;
  bool is_manual_ctrl_enabled_ = false;
  uint8_t mission_priority_ = tobas_mission_msgs::msg::Priority::NORMAL;
  std::unique_ptr<kdl::Vector> launch_point_;

  enum Status
  {
    kNoProblem,
    kMissionSuperseded,
    kManualOverride,
  } status_ = kNoProblem;

  /* Target trajectory point expressed in the global frame. */
  struct Command
  {
    kdl::Vector pos;
    kdl::Vector vel;
    kdl::Vector acc;
    kdl::Euler rot;
  } command_;

  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_;
  tobas_msgs::OdometryStamped::ConstSharedPtr setpoint_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr gnss_origin_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;

  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAcc> pva_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccYaw> pvay_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccPitchYaw> pvapy_pub_;

  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryStamped> setpoint_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::GeodeticCoordinates> gnss_origin_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;

  ros2::ActionServerPtr<Action> as_;

  void getStaticRosParams();

  /* 設定値が存在すれば設定値，存在しなければ現在値でコマンドを初期化する． */
  void initializeCommand();

  /* コマンドを発行する． */
  void publishCommand(const rclcpp::Time& stamp);

  /* アーム・ディスアーム要求を行う． */
  bool armRotors(bool arming);

  /* 現在のコマンドから滑らかに停止させる． */
  void brake();

  /**
   * @brief 外部からの要求（キャンセル，上書きなど）に応じて適切に終了処理を行う．
   * @return bool ミッション継続可能かどうか
   */
  bool handleExternalRequest(const GoalHandlePtr& gh, const ResultPtr& res);

  bool executeWaypoint(const Waypoint& goal, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeTakeoff(const Takeoff& goal, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeLand(const Land& goal, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeRTL(const ReturnToLaunch& goal, const GoalHandlePtr& gh, const ResultPtr& res);

  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void setpointCb(const tobas_msgs::OdometryStamped::ConstSharedPtr& setpoint);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, const GoalPtr& goal);
  rclcpp_action::CancelResponse handleCancel(const GoalHandlePtr& gh);
  void execute(const GoalHandlePtr& gh);
};

MulticopterMissionExecutorNode::MulticopterMissionExecutorNode(const rclcpp::NodeOptions& options)
  : super(node::kMissionExecutor, nodeOptions_Default(options))
{
  getStaticRosParams();

  angle_pub_ = createPublisher<tobas_command_msgs::Angle>(topic::kAngleCmd);
  pva_pub_ = createPublisher<tobas_command_msgs::PosVelAcc>(topic::kPosVelAccCmd);
  pvay_pub_ = createPublisher<tobas_command_msgs::PosVelAccYaw>(topic::kPosVelAccYawCmd);
  pvapy_pub_ = createPublisher<tobas_command_msgs::PosVelAccPitchYaw>(topic::kPosVelAccPitchYawCmd);

  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
  setpoint_sub_ = createSubscriber(topic::kTrajSetpoint, &self::setpointCb, this);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  gnss_origin_sub_ = createSubscriber(topic::kGnssOrigin, &self::gnssOriginCb, this, true, true);
  landed_sub_ = createSubscriber(topic::kLanded, &self::landedCb, this);
  rcin_sub_ = createSubscriber(topic::kRcInput, &self::rcInputCb, this);

  as_ = createAction(action::kExecuteMission, &self::handleGoal, &self::handleCancel, &self::execute, this);
}

void MulticopterMissionExecutorNode::getStaticRosParams()
{
  wp_cfg_.max_hor_vel = getDoubleParam("waypoint/max_horizontal_velocity");
  wp_cfg_.max_hor_acc = getDoubleParam("waypoint/max_horizontal_accel");
  wp_cfg_.max_hor_jerk = getDoubleParam("waypoint/max_horizontal_jerk");
  wp_cfg_.max_ver_vel = getDoubleParam("waypoint/max_vertical_velocity");
  wp_cfg_.max_ver_acc = getDoubleParam("waypoint/max_vertical_accel");
  wp_cfg_.max_ver_jerk = getDoubleParam("waypoint/max_vertical_jerk");
  wp_cfg_.max_head_rate = getDoubleParam("waypoint/max_heading_rate");
  wp_cfg_.max_head_acc = getDoubleParam("waypoint/max_heading_accel");

  takeoff_cfg_.max_speed = getDoubleParam("takeoff/max_speed");
  takeoff_cfg_.max_accel = getDoubleParam("takeoff/max_accel");
  takeoff_cfg_.max_jerk = getDoubleParam("takeoff/max_jerk");

  land_cfg_.speed = getDoubleParam("land/speed");

  rtl_cfg_.min_alt = getDoubleParam("rtl/min_altitude");
}

void MulticopterMissionExecutorNode::initializeCommand()
{
  const auto& odom = odom_->odom.odom;

  if (setpoint_) {
    const auto& sp = setpoint_->odom;
    if (sp.frame.p.isFinite()) {
      command_.pos = sp.frame.p;
    }
    else {
      command_.pos = odom.frame.p;
    }
    if (sp.frame.M.isFinite()) {
      sp.frame.M.getRPY(command_.rot.roll, command_.rot.pitch, command_.rot.yaw);
    }
    else {
      odom.frame.M.getRPY(command_.rot.roll, command_.rot.pitch, command_.rot.yaw);
    }
    if (sp.twist.vel.isFinite()) {
      command_.vel = odom.frame.M * sp.twist.vel;
    }
    else {
      command_.vel = odom.frame.M * odom.twist.vel;
    }
    if (sp.accel.linear.isFinite()) {
      command_.acc = odom.frame.M * sp.accel.linear;
    }
    else {
      command_.acc = odom.frame.M * odom.accel.linear;
    }
  }
  else {
    command_.pos = odom.frame.p;
    odom.frame.M.getRPY(command_.rot.roll, command_.rot.pitch, command_.rot.yaw);
    command_.vel = odom.twist.vel;
    command_.acc = odom.accel.linear;
  }
}

void MulticopterMissionExecutorNode::publishCommand(const rclcpp::Time& stamp)
{
  // ミッション優先度に応じてコマンド優先度を設定
  uint8_t cmd_priority;
  switch (mission_priority_) {
    case tobas_mission_msgs::msg::Priority::NORMAL:
      cmd_priority = tobas_command_msgs::msg::Priority::NORMAL;
      break;
    case tobas_mission_msgs::msg::Priority::DEFENSIVE:
      cmd_priority = tobas_command_msgs::msg::Priority::DEFENSIVE;
      break;
    default:
      TOBAS_ERROR("Invalid mission priority: ", (int)mission_priority_);
      return;
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::Angle>();
    cmd->header.stamp = stamp;
    cmd->priority.data = cmd_priority;
    cmd->angle = command_.rot;
    angle_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVelAcc>();
    cmd->header.stamp = stamp;
    cmd->priority.data = cmd_priority;
    cmd->pos = command_.pos;
    cmd->vel = command_.vel;
    cmd->acc = command_.acc;
    pva_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVelAccYaw>();
    cmd->header.stamp = stamp;
    cmd->priority.data = cmd_priority;
    cmd->pos = command_.pos;
    cmd->vel = command_.vel;
    cmd->acc = command_.acc;
    cmd->yaw = command_.rot.yaw;
    pvay_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVelAccPitchYaw>();
    cmd->header.stamp = stamp;
    cmd->priority.data = cmd_priority;
    cmd->pos = command_.pos;
    cmd->vel = command_.vel;
    cmd->acc = command_.acc;
    cmd->pitch = command_.rot.pitch;
    cmd->yaw = command_.rot.yaw;
    pvapy_pub_->publish(std::move(cmd));
  }
}

bool MulticopterMissionExecutorNode::armRotors(bool arming)
{
  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), service::kSetArm);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  if (!sc.call(req)) {
    TOBAS_ERROR("Failed to call \"", service::kSetArm, "\" service.");
    return false;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    TOBAS_ERROR("Failed to set the arming status: ", res->message);
    return false;
  }

  return true;
}

void MulticopterMissionExecutorNode::brake()
{
  // 軌道を生成
  const Eigen::Vector2d pxy0(command_.pos.x(), command_.pos.y());
  const Eigen::Vector2d vxy0(command_.vel.x(), command_.vel.y());
  const Eigen::Vector2d axy0(command_.acc.x(), command_.acc.y());
  const auto vxy0_norm = vxy0.norm();
  const auto axy0_norm = axy0.norm();
  const auto dir_xy = vxy0_norm > 0. ? (vxy0 / vxy0_norm).eval() : Eigen::Vector2d::Zero();
  const StopTrajectory traj_xy(
    0., vxy0_norm, axy0_norm * math::sign(vxy0.dot(axy0)), wp_cfg_.max_hor_acc, wp_cfg_.max_hor_jerk);

  const auto pz0 = command_.pos.z();  // Must be copy
  const auto vz0 = command_.vel.z();  // Must be copy
  const auto az0 = command_.acc.z();  // Must be copy
  const auto vz0_norm = std::abs(vz0);
  const auto az0_norm = std::abs(az0);
  const auto dir_z = math::sign(vz0);
  const StopTrajectory traj_z(0., vz0_norm, az0_norm * math::sign(vz0 * az0), wp_cfg_.max_ver_acc, wp_cfg_.max_ver_jerk);

  // 所要時間を取得
  const auto duration = std::max(traj_xy.duration(), traj_z.duration());
  if (duration < kMinBrakeDuration) {
    return;  // 既にほぼ停止している場合はコマンドを発行せず終了
  }
  TOBAS_INFO("The vehicle will stop in ", duration, " seconds.");

  // 軌道を発行
  const auto start_time = now();
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // 開始からの経過時間を計算
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();

    // コマンドを発行し終えたら終了
    if (t > duration) {
      return;
    }

    // 現在の時刻における目標状態を取得
    const auto traj_point_xy = traj_xy.get(t);
    const Eigen::Vector2d pxy = pxy0 + traj_point_xy.p * dir_xy;
    const Eigen::Vector2d vxy = traj_point_xy.v * dir_xy;
    const Eigen::Vector2d axy = traj_point_xy.a * dir_xy;
    const auto traj_point_z = traj_z.get(t);
    const auto pz = pz0 + traj_point_z.p * dir_z;
    const auto vz = traj_point_z.v * dir_z;
    const auto az = traj_point_z.a * dir_z;
    command_.pos.set(pxy.x(), pxy.y(), pz);
    command_.vel.set(vxy.x(), vxy.y(), vz);
    command_.acc.set(axy.x(), axy.y(), az);

    // コマンドを発行
    publishCommand(cur_time);

    rate.sleep();
  }
}

bool MulticopterMissionExecutorNode::handleExternalRequest(const GoalHandlePtr& gh, const ResultPtr& res)
{
  // アクション中止の場合は滑らかに停止して終了
  if (gh->is_canceling()) {
    brake();
    gh->canceled(res);
    return false;
  }

  switch (status_) {
    case kNoProblem:
      return true;
    case kMissionSuperseded:
      brake();
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::MISSION_SUPERSEDED;
      res->error_message = "The mission was superseded by a new mission.";
      gh->abort(res);
      return false;
    case kManualOverride:
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::MANUAL_OVERRIDE;
      res->error_message = "Autopilot was overridden by manual control";
      gh->abort(res);
      return false;
    default:
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
      res->error_message = "Invalid status: " + std::to_string((int)status_);
      gh->abort(res);
      return false;
  }
}

bool MulticopterMissionExecutorNode::executeWaypoint(const Waypoint& goal, const GoalHandlePtr& gh, const ResultPtr& res)
{
  // Verify that the vehicle is armed
  if (!arming_->data) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "The waypoint mission cannot be started because the vehicle is disarmed.";
    gh->abort(res);
    return false;
  }

  // 開始前に一時停止
  brake();

  // 目標状態の初期値を取得
  const auto start_pos = command_.pos.clone();
  const auto start_rot = command_.rot.clone();

  // 目標位置を計算
  kdl::Vector goal_pos;  // wrt. the odometry frame
  std::tie(goal_pos.x(), goal_pos.y()) =
    st::gnssToCartRelative(goal.latitude, goal.longitude, gnss_origin_->latitude, gnss_origin_->longitude);
  switch (goal.altitude_frame) {
    case kRelativeToLaunch:
      if (!launch_point_) {
        res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
        res->error_message = "Launch point is not set.";
        gh->abort(res);
        return false;
      }
      goal_pos.z(goal.altitude + launch_point_->z());
      break;
    case kMeanSeaLevel:  // TODO
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
      res->error_message = "Not implemented yet.";
      gh->abort(res);
      return false;
    default:
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
      res->error_message = "Invalid altitude frame type.";
      gh->abort(res);
      return false;
  }
  TOBAS_INFO("Goal position: ", goal_pos);

  // 制約を決定
  const auto max_hor_vel = goal.max_horizontal_velocity > 0. ? goal.max_horizontal_velocity : wp_cfg_.max_hor_vel;
  const auto max_hor_acc = goal.max_horizontal_accel > 0. ? goal.max_horizontal_accel : wp_cfg_.max_hor_acc;
  const auto max_hor_jerk = goal.max_horizontal_jerk > 0. ? goal.max_horizontal_jerk : wp_cfg_.max_hor_jerk;
  const auto max_ver_vel = goal.max_vertical_velocity > 0. ? goal.max_vertical_velocity : wp_cfg_.max_ver_vel;
  const auto max_ver_acc = goal.max_vertical_accel > 0. ? goal.max_vertical_accel : wp_cfg_.max_ver_acc;
  const auto max_ver_jerk = goal.max_vertical_jerk > 0. ? goal.max_vertical_jerk : wp_cfg_.max_ver_jerk;
  const auto max_head_rate = goal.max_heading_rate > 0. ? goal.max_heading_rate : wp_cfg_.max_head_rate;
  const auto max_head_acc = goal.max_heading_accel > 0. ? goal.max_heading_accel : wp_cfg_.max_head_acc;

  // 軌道を生成
  const Eigen::Vector2d start_xy(start_pos.x(), start_pos.y());
  const Eigen::Vector2d goal_xy(goal_pos.x(), goal_pos.y());
  const Eigen::Vector2d xy_diff = goal_xy - start_xy;  // XYの軌道を別々に生成すると最短経路を通らないことに注意
  const auto xy_dist = xy_diff.norm();  // [m]
  if (xy_dist == 0.) {
    return true;
  }
  const Eigen::Vector2d xy_dir = xy_diff / xy_dist;
  const traj::TimeOptimalTrajectory traj_xy(0., xy_dist, max_hor_jerk, max_hor_acc, max_hor_vel);

  const traj::TimeOptimalTrajectory traj_z(start_pos.z(), goal_pos.z(), max_ver_jerk, max_ver_acc, max_ver_vel);

  const auto roll_duration = std::abs(start_rot.roll) / kAttitudeRate;
  const traj::LinearSpline traj_roll(start_rot.roll, 0., roll_duration);

  const auto pitch_duration = std::abs(start_rot.pitch) / kAttitudeRate;
  const traj::LinearSpline traj_pitch(start_rot.pitch, 0., pitch_duration);

  const auto goal_yaw = goal.auto_heading ? atan2(xy_dir.y(), xy_dir.x()) : start_rot.yaw;
  const auto yaw_diff = algo::wrapPi(goal_yaw - start_rot.yaw);  // 最短経路をとるよう[-π, π)の範囲に変換
  const traj::TimeOptimalTrajectory traj_yaw(0., yaw_diff, INFINITY, max_head_acc, max_head_rate);

  // 所要時間を取得
  const auto pos_duration = std::max(traj_xy.duration(), traj_z.duration());
  const auto rot_duration = algo::max(traj_roll.duration(), traj_pitch.duration(), traj_yaw.duration());
  const auto duration = std::max(pos_duration, rot_duration);
  TOBAS_INFO("Moving to the target position will take ", duration, " seconds.");

  // 軌道を発行
  const auto start_time = now();
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // ミッション継続可能かどうかを確認
    if (!handleExternalRequest(gh, res)) {
      return false;
    }

    // 開始からの経過時間を計算
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();

    // タイムアウトの確認
    if (goal.timeout > 0. && t > duration + goal.timeout) {
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::ACCEPTANCE_TIMEOUT;
      res->error_message = "Timed out before reaching the waypoint acceptance radius.";
      gh->abort(res);
      return false;
    }

    // 現在の位置を取得
    const auto& cur_pos = odom_->odom.odom.frame.p;

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
    const Eigen::Vector2d axy = traj_point_xy.a * xy_dir;
    const auto traj_point_z = traj_z.get(t);
    command_.pos.set(pxy.x(), pxy.y(), traj_point_z.p);
    command_.vel.set(vxy.x(), vxy.y(), traj_point_z.v);
    command_.acc.set(axy.x(), axy.y(), traj_point_z.a);
    command_.rot.set(traj_roll.get(t).p, traj_pitch.get(t).p, algo::wrapPi(start_rot.yaw + traj_yaw.get(t).p));

    // コマンドを発行
    publishCommand(cur_time);

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeTakeoff(const Takeoff& goal, const GoalHandlePtr& gh, const ResultPtr& res)
{
  // Verify that the vehicle is disarmed
  if (arming_->data) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "The takeoff mission cannot be started because the vehicle is already armed.";
    gh->abort(res);
    return false;
  }

  // Arm rotors
  if (!armRotors(true)) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "Failed to arm the vehicle.";
    gh->abort(res);
    return false;
  }

  // TODO: 正常にアームされたかどうかを確認

  // 目標状態の初期値を取得
  const auto start_pos = command_.pos.clone();
  const auto start_yaw = command_.rot.yaw;

  // 目標高度を決定
  double tar_z;  // wrt. the odometry frame
  switch (goal.altitude_frame) {
    case kRelativeToLaunch:
      tar_z = start_pos.z() + goal.altitude;
      break;
    case kMeanSeaLevel:  // TODO
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
      res->error_message = "Not implemented yet.";
      gh->abort(res);
      return false;
    default:
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
      res->error_message = "Invalid altitude frame type.";
      gh->abort(res);
      return false;
  }

  // 制約を決定
  const auto max_speed = goal.max_speed > 0. ? goal.max_speed : takeoff_cfg_.max_speed;
  const auto max_accel = goal.max_accel > 0. ? goal.max_accel : takeoff_cfg_.max_accel;
  const auto max_jerk = goal.max_jerk > 0. ? goal.max_jerk : takeoff_cfg_.max_jerk;

  // 軌道を生成
  const traj::TimeOptimalTrajectory traj_z(start_pos.z(), tar_z, max_jerk, max_accel, max_speed);

  // 所要時間を取得
  const auto duration = traj_z.duration();
  TOBAS_INFO("Takeoff will take ", duration, " seconds.");

  // 軌道を発行
  const auto start_time = now();
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // ミッション継続可能かどうかを確認
    if (!handleExternalRequest(gh, res)) {
      return false;
    }

    // 開始からの経過時間を計算
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();

    // タイムアウトの確認
    if (goal.timeout > 0. && t > duration + goal.timeout) {
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::ACCEPTANCE_TIMEOUT;
      res->error_message = "Timed out before reaching the takeoff altitude tolerance.";
      gh->abort(res);
      return false;
    }

    // コマンドを発行し終え，且つ許容範囲内に入っていたらアクション成功
    const auto& cur_pos = odom_->odom.odom.frame.p;
    const auto alt_err_abs = std::abs(tar_z - cur_pos.z());
    if (t > duration) {
      if (goal.altitude_tolerance <= 0. || alt_err_abs < goal.altitude_tolerance) {
        return true;
      }
    }

    // コマンドを作成
    const auto traj_point_z = traj_z.get(t);
    command_.pos.set(start_pos.x(), start_pos.y(), traj_point_z.p);
    command_.vel.set(0., 0., traj_point_z.v);
    command_.acc.set(0., 0., traj_point_z.a);
    command_.rot.set(0., 0., start_yaw);

    // コマンドを発行
    publishCommand(cur_time);

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeLand(const Land& goal, const GoalHandlePtr& gh, const ResultPtr& res)
{
  // Verify that the vehicle is armed
  if (!arming_->data) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "The land mission cannot be started because the vehicle is disarmed.";
    gh->abort(res);
    return false;
  }

  // 開始前に一時停止
  brake();

  // 目標状態の初期値を取得
  const auto start_pos = command_.pos.clone();
  const auto start_rot = command_.rot.clone();

  // 下降速度を決定
  const auto speed = goal.speed > 0. ? goal.speed : land_cfg_.speed;

  // 姿勢の起動を生成
  const auto roll_duration = std::abs(start_rot.roll) / kAttitudeRate;
  const auto pitch_duration = std::abs(start_rot.pitch) / kAttitudeRate;
  const traj::LinearSpline traj_roll(start_rot.roll, 0., roll_duration);
  const traj::LinearSpline traj_pitch(start_rot.pitch, 0., pitch_duration);

  // 着陸判定に使うオブジェクトを作成
  const auto stop_speed_thresh = std::min<double>(speed / 2, 0.2);
  auto t_last_high_speed = odom_->header.stamp;

  // 姿勢を戻しながら下降
  const auto start_time = now();
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // ミッション継続可能かどうかを確認
    if (!handleExternalRequest(gh, res)) {
      return false;
    }

    // 現在時刻における目標位置姿勢を計算
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();
    const auto tar_z = start_pos.z() - speed * t;
    command_.pos.set(start_pos.x(), start_pos.y(), tar_z);
    command_.vel.set(0., 0., -speed);
    command_.acc.setZero();
    command_.rot.set(traj_roll.get(t).p, traj_pitch.get(t).p, start_rot.yaw);

    // コマンドを発行
    publishCommand(cur_time);

    // 最新のIMU時刻を取得
    const auto imu_time = odom_->header.stamp;  // Copy

    // 鉛直方向の速度を計算
    const auto cur_vel_W = odom_->odom.odom.frame.M * odom_->odom.odom.twist.vel;
    const auto& cur_vz = cur_vel_W.z();

    // 最後に高い速度を検知した時刻からの経過時間を計算
    if (std::abs(cur_vz) > stop_speed_thresh) {
      t_last_high_speed = imu_time;
    }
    const auto time_from_last_high_speed = imu_time - t_last_high_speed;

    // 高度誤差を計算
    const auto z_error = tar_z - odom_->odom.odom.frame.p.z();

    // 以下の条件のうちいずれか1つが満たされたらモータを停止して終了
    // 1. 自重に近い地面反力を検知（共通の着陸検知アルゴリズム）
    // 2. 鉛直方向の速度の絶対値が小さい状態が一定時間持続: https://ardupilot.org/copter/docs/land-mode.html
    // 3. 目標高度と推定高度の差が非常に大きい（どうしても他の条件が満たされない場合の最後の手段）
    if (landed_->landed || time_from_last_high_speed > 1s || z_error < -10.) {
      TOBAS_INFO("Landing detected. Stopping motors.");
      if (!armRotors(false)) {
        res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
        res->error_message = "Failed to disarm rotors.";
        gh->abort(res);
        return false;
      }
      return true;
    }

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeRTL(const ReturnToLaunch& goal, const GoalHandlePtr& gh, const ResultPtr& res)
{
  // cf. [Return Mode | PX4](https://docs.px4.io/main/en/flight_modes/return)

  if (!launch_point_) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "The RTL mission cannot be started because the launch point is not set.";
    gh->abort(res);
    return false;
  }

  // 開始前に一時停止
  brake();

  // ウェイポイントのゴールを作成
  Waypoint wp;
  wp.altitude_frame = AltitudeFrame::kRelativeToLaunch;
  wp.max_horizontal_velocity = goal.max_horizontal_velocity;
  wp.max_horizontal_accel = goal.max_horizontal_accel;
  wp.max_vertical_velocity = goal.max_vertical_velocity;
  wp.max_horizontal_jerk = goal.max_horizontal_jerk;
  wp.max_vertical_accel = goal.max_vertical_accel;
  wp.max_vertical_jerk = goal.max_vertical_jerk;
  wp.max_heading_rate = goal.max_heading_rate;
  wp.max_heading_accel = goal.max_heading_accel;
  wp.acceptance_radius = goal.acceptance_radius;
  wp.altitude_tolerance = goal.altitude_tolerance;
  wp.timeout = goal.timeout;

  // 目標高度を決定
  const auto& cur_pos = odom_->odom.odom.frame.p;
  const auto cur_alt = cur_pos.z() - launch_point_->z();
  const auto xy_dist = math::norm(launch_point_->x() - cur_pos.x(), launch_point_->y() - cur_pos.y());
  const auto min_alt_goal = goal.min_altitude > 0. ? goal.min_altitude : rtl_cfg_.min_alt;
  const auto min_alt = std::min<double>(min_alt_goal, xy_dist);  // 45度逆円錐ルール
  wp.altitude = std::max(cur_alt, min_alt);

  // 現在の高度がRTLの最低高度よりも低い場合はそこまで上昇
  if (cur_alt < min_alt) {
    const auto [tar_lat, tar_lon] =
      st::cartToGnssRelative(cur_pos.x(), cur_pos.y(), gnss_origin_->latitude, gnss_origin_->longitude);
    wp.latitude = tar_lat;
    wp.longitude = tar_lon;
    wp.auto_heading = false;
    if (!executeWaypoint(wp, gh, res)) {
      return false;
    }
  }

  // アームした地点まで移動
  const auto [tar_lat, tar_lon] =
    st::cartToGnssRelative(launch_point_->x(), launch_point_->y(), gnss_origin_->latitude, gnss_origin_->longitude);
  wp.latitude = tar_lat;
  wp.longitude = tar_lon;
  wp.auto_heading = true;
  if (!executeWaypoint(wp, gh, res)) {
    return false;
  }

  // 着陸
  Land land;
  land.timeout = goal.timeout;
  if (!executeLand(land, gh, res)) {
    return false;
  }

  return true;
}

void MulticopterMissionExecutorNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void MulticopterMissionExecutorNode::setpointCb(const tobas_msgs::OdometryStamped::ConstSharedPtr& setpoint)
{
  setpoint_ = setpoint;
}

void MulticopterMissionExecutorNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  if (!arming_) {
    arming_ = arming;
    return;
  }

  // アームされた座標を保存
  if (!arming_->data && arming->data) {
    if (odom_) {
      launch_point_ = std::make_unique<kdl::Vector>(odom_->odom.odom.frame.p);
    }
  }

  // ディスアームされたらアーム座標と設定値をリセット
  if (arming_->data && !arming->data) {
    launch_point_.reset();
    setpoint_.reset();
  }

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

void MulticopterMissionExecutorNode::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  is_manual_ctrl_enabled_ = (rcin->ok && rcin->enable);

  if (is_executing_ && is_manual_ctrl_enabled_) {
    status_ = kManualOverride;
  }
}

rclcpp_action::GoalResponse
MulticopterMissionExecutorNode::handleGoal(const rclcpp_action::GoalUUID&, const GoalPtr& goal)
{
  TOBAS_INFO("New mission is uploaded.");

  // Check mission priority
  const auto& new_priority = goal->priority.data;
  if (new_priority > tobas_mission_msgs::msg::Priority::DEFENSIVE) {
    TOBAS_WARN("Invalid mission priority: ", (int)new_priority);
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (is_executing_) {
    if (new_priority < mission_priority_) {
      TOBAS_WARN("The mission cannot be executed because its priority is lower than the one currently being executed.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  // Check the essential topics
  if (!odom_) {
    TOBAS_WARN("Odometry has not been received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!arming_) {
    TOBAS_WARN("Arming status has not been received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!gnss_origin_) {
    TOBAS_WARN("GNSS origin has not been received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!landed_) {
    TOBAS_WARN("Landed state has not been received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Reject the mission if manual control is enabled
  if (is_manual_ctrl_enabled_) {
    TOBAS_WARN("Mission cannot be executed while manual control is enabled.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check the mission items
  for (const auto& [idx, item] : std::views::enumerate(goal->items)) {
    switch (item.type) {
      case kWaypoint: {
        Waypoint waypoint;
        if (!st::fromBytes(item.data, waypoint)) {
          TOBAS_ERROR("Mission No. ", idx, ": Size mismatch.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (waypoint.latitude < -90 || 90 < waypoint.latitude) {
          TOBAS_ERROR("Mission No. ", idx, ": Invalid target latitude.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.longitude < -180 || 180 < waypoint.longitude) {
          TOBAS_ERROR("Mission No. ", idx, ": Invalid target longitude.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        break;
      }
      case kTakeoff: {
        Takeoff takeoff;
        if (!st::fromBytes(item.data, takeoff)) {
          TOBAS_ERROR("Mission No. ", idx, ": Size mismatch.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (takeoff.altitude <= 0.) {
          TOBAS_ERROR("Mission No. ", idx, ": Target altitude must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        break;
      }
      case kLand: {
        Land land;
        if (!st::fromBytes(item.data, land)) {
          TOBAS_ERROR("Mission No. ", idx, ": Size mismatch.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        break;
      }
      case kReturnToLaunch: {
        ReturnToLaunch rtl;
        if (!st::fromBytes(item.data, rtl)) {
          TOBAS_ERROR("Mission No. ", idx, ": Size mismatch.");
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

  // ミッション優先度を更新
  mission_priority_ = new_priority;

  // 古いミッションが実行中なら中断要求
  if (is_executing_) {
    status_ = kMissionSuperseded;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MulticopterMissionExecutorNode::handleCancel(const GoalHandlePtr&)
{
  TOBAS_INFO("Mission cancellation has been requested.");

  if (!is_executing_) {
    TOBAS_ERROR("No mission is in execution.");
    return rclcpp_action::CancelResponse::REJECT;
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void MulticopterMissionExecutorNode::execute(const GoalHandlePtr& gh)
{
  // Wait until the previous mission is finished
  rclcpp::Rate rate(kCommandRate);
  while (rclcpp::ok() && is_executing_) {
    rate.sleep();
  }

  // Now the new mission is in execution
  is_executing_ = true;
  status_ = kNoProblem;

  // Initialize the command
  initializeCommand();

  // Create result
  const auto res = std::make_shared<Result>();

  // Get goal
  const auto goal = gh->get_goal();

  // Execute mission
  for (const auto& [idx, item] : std::views::enumerate(goal->items)) {
    TOBAS_INFO("Start mission No. ", idx);

    // Publish the current mission number
    const auto feedback = std::make_shared<Action::Feedback>();
    feedback->current_index = idx;
    gh->publish_feedback(feedback);

    switch (item.type) {
      case kWaypoint: {
        Waypoint waypoint;
        st::fromBytes(item.data, waypoint);
        if (!executeWaypoint(waypoint, gh, res)) {
          is_executing_ = false;
          return;
        }
        break;
      }
      case kTakeoff: {
        Takeoff takeoff;
        st::fromBytes(item.data, takeoff);
        if (!executeTakeoff(takeoff, gh, res)) {
          is_executing_ = false;
          return;
        }
        break;
      }
      case kLand: {
        Land land;
        st::fromBytes(item.data, land);
        if (!executeLand(land, gh, res)) {
          is_executing_ = false;
          return;
        }
        break;
      }
      case kReturnToLaunch: {
        ReturnToLaunch rtl;
        st::fromBytes(item.data, rtl);
        if (!executeRTL(rtl, gh, res)) {
          is_executing_ = false;
          return;
        }
        break;
      }
      default: {
        res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
        res->error_message = "Invalid mission type: " + std::to_string(idx);
        gh->abort(res);
        is_executing_ = false;
        return;
      }
    }
  }

  res->error_code.data = tobas_mission_msgs::msg::ErrorCode::NO_ERROR;
  res->error_message.clear();
  gh->succeed(res);
  is_executing_ = false;
}
}  // namespace mission
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::mission::MulticopterMissionExecutorNode)
