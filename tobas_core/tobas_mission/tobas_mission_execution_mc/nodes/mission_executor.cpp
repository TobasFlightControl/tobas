#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_math/linalg.hpp>
#include <tobas_mission_items/mission_items.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_std_tools/gnss.hpp>
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
#include <tobas_msgs_adapter/gnss.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

namespace tobas
{
struct GeoPoint
{
  using SharedPtr = std::shared_ptr<GeoPoint>;

  double latitude;   // [deg]
  double longitude;  // [deg]
};

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

  enum Status
  {
    kNoProblem,
    kMissionSuperseded,
    kManualOverride,
  } status_ = kNoProblem;

  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::Gnss::ConstSharedPtr gnss_;
  tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr gnss_origin_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;

  GeoPoint::SharedPtr launch_point_;
  GeoPoint::SharedPtr last_setpoint_;

  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVel> pos_vel_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelYaw> pos_vel_yaw_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelPitchYaw> pos_vel_pitch_yaw_pub_;

  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::Gnss> gnss_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::GeodeticCoordinates> gnss_origin_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;

  ros2::ActionServerPtr<Action> as_;

  void getStaticRosParams();
  void publishCommands(
    const rclcpp::Time& stamp,
    const kdl::Vector& pos,
    const kdl::Vector& vel,
    double roll,
    double pitch,
    double yaw);
  bool armRotors(bool arming);
  bool checkCurrentStatus(const GoalHandlePtr& gh, const ResultPtr& res);

  bool executeWaypoint(const Waypoint& goal, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeTakeoff(const Takeoff& goal, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeLand(const Land& goal, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeRTL(const ReturnToLaunch& goal, const GoalHandlePtr& gh, const ResultPtr& res);

  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
  void gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID& uuid, const GoalPtr& goal);
  rclcpp_action::CancelResponse handleCancel(const GoalHandlePtr& gh);
  void execute(const GoalHandlePtr& gh);
};

MulticopterMissionExecutorNode::MulticopterMissionExecutorNode(const rclcpp::NodeOptions& options)
  : super(tobas::node::kMissionExecutor, options)
{
  getStaticRosParams();

  angle_pub_ = createPublisher<tobas_command_msgs::Angle>(kAngleCmdTopic);
  pos_vel_pub_ = createPublisher<tobas_command_msgs::PosVel>(kPosVelCmdTopic);
  pos_vel_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelYaw>(kPosVelYawCmdTopic);
  pos_vel_pitch_yaw_pub_ = createPublisher<tobas_command_msgs::PosVelPitchYaw>(kPosVelPitchYawCmdTopic);

  odom_sub_ = createSubscriber(kOdometryTopic, &self::odomCb, this);
  arming_sub_ = createSubscriber(kArmingTopic, &self::armingCb, this);
  gnss_sub_ = createSubscriber(kGnssTopic, &self::gnssCb, this);
  gnss_origin_sub_ = createSubscriber(kGnssOriginTopic, &self::gnssOriginCb, this, true, true);
  landed_sub_ = createSubscriber(kLandedTopic, &self::landedCb, this);
  rcin_sub_ = createSubscriber(kRcInputTopic, &self::rcInputCb, this);

  as_ = createAction(kExecuteMissionAction, &self::handleGoal, &self::handleCancel, &self::execute, this);
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

void MulticopterMissionExecutorNode::publishCommands(
  const rclcpp::Time& stamp,
  const kdl::Vector& pos,
  const kdl::Vector& vel,
  double roll,
  double pitch,
  double yaw)
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
    cmd->angle.roll = roll;
    cmd->angle.pitch = pitch;
    cmd->angle.yaw = yaw;
    angle_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVel>();
    cmd->header.stamp = stamp;
    cmd->priority.data = cmd_priority;
    cmd->pos = pos;
    cmd->vel = vel;
    pos_vel_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVelYaw>();
    cmd->header.stamp = stamp;
    cmd->priority.data = cmd_priority;
    cmd->pos = pos;
    cmd->vel = vel;
    cmd->yaw = yaw;
    pos_vel_yaw_pub_->publish(std::move(cmd));
  }

  {
    auto cmd = std::make_unique<tobas_command_msgs::PosVelPitchYaw>();
    cmd->header.stamp = stamp;
    cmd->priority.data = cmd_priority;
    cmd->pos = pos;
    cmd->vel = vel;
    cmd->pitch = pitch;
    cmd->yaw = yaw;
    pos_vel_pitch_yaw_pub_->publish(std::move(cmd));
  }
}

bool MulticopterMissionExecutorNode::armRotors(bool arming)
{
  ros2::SyncServiceClient<tobas_msgs::srv::SetArm> sc(shared_from_this(), kSetArmSrv);

  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;
  if (!sc.call(req)) {
    TOBAS_ERROR("Failed to call \"", kSetArmSrv, "\" service.");
    return false;
  }

  const auto res = sc.getResponse();
  if (!res->success) {
    TOBAS_ERROR("Failed to set the arming status: ", res->message);
    return false;
  }

  return true;
}

bool MulticopterMissionExecutorNode::checkCurrentStatus(const GoalHandlePtr& gh, const ResultPtr& res)
{
  switch (status_) {
    case kNoProblem:
      return true;
    case kMissionSuperseded:
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
  // Verify that GNSS is fixed
  if (gnss_->fix_type != tobas_msgs::msg::Gnss::FIX_3D) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "GNSS is lost.";
    gh->abort(res);
    return false;
  }

  // Verify that the vehicle is armed
  if (!arming_->data) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "The vehicle is disarmed.";
    gh->abort(res);
    return false;
  }

  // Save the latest setpoint
  last_setpoint_ = std::make_shared<GeoPoint>(goal.latitude, goal.longitude);

  // 初期状態を取得
  const auto start_time = now();
  const auto start_pos = odom_->frame.p.clone();
  const kdl::Euler start_rpy(odom_->frame.M);
  TOBAS_INFO("Start position: ", start_pos);

  // 目標位置を計算
  kdl::Vector goal_pos;
  std::tie(goal_pos.x(), goal_pos.y()) =
    tbs::gnssToCartRelative(goal.latitude, goal.longitude, gnss_origin_->latitude, gnss_origin_->longitude);
  goal_pos.z(goal.altitude);  // TODO: 目標高度がMSLで与えられた場合にも対応
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
  Eigen::Vector2d start_xy(start_pos.x(), start_pos.y());
  const Eigen::Vector2d goal_xy(goal_pos.x(), goal_pos.y());
  const Eigen::Vector2d xy_diff = goal_xy - start_xy;  // XYの軌道を別々に生成すると最短経路を通らないことに注意
  const auto xy_dist = xy_diff.norm();  // [m]
  if (xy_dist == 0.) {
    return true;
  }
  const Eigen::Vector2d xy_dir = xy_diff / xy_dist;
  const traj::TimeOptimalTrajectory traj_xy(0., xy_dist, max_hor_jerk, max_hor_acc, max_hor_vel);

  // TODO: Altitude Frame を考慮
  const traj::TimeOptimalTrajectory traj_z(start_pos.z(), goal_pos.z(), max_ver_jerk, max_ver_acc, max_ver_vel);

  const auto roll_duration = std::abs(start_rpy.roll) / kAttitudeRate;
  const traj::LinearSpline traj_roll(start_rpy.roll, 0., roll_duration);

  const auto pitch_duration = std::abs(start_rpy.pitch) / kAttitudeRate;
  const traj::LinearSpline traj_pitch(start_rpy.pitch, 0., pitch_duration);

  const auto goal_yaw = goal.auto_heading ? atan2(xy_dir.y(), xy_dir.x()) : start_rpy.yaw;
  const auto yaw_diff = algo::wrapPi(goal_yaw - start_rpy.yaw);  // 最短経路をとるよう[-π, π)の範囲に変換
  const traj::TimeOptimalTrajectory traj_yaw(0., yaw_diff, INFINITY, max_head_acc, max_head_rate);

  // 所要時間を取得
  const auto duration = std::max(traj_xy.duration(), traj_z.duration());
  TOBAS_INFO("Moving to the target position will take ", duration, " seconds.");

  // 軌道を発行
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // ミッション継続可能かどうかを確認
    if (!checkCurrentStatus(gh, res)) {
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
    if (gh->is_canceling()) {
      tar_vel.setZero();
    }

    // コマンドを発行
    publishCommands(cur_time, tar_pos, tar_vel, tar_roll, tar_pitch, tar_yaw);

    // アクション中止の場合は終了
    if (gh->is_canceling()) {
      gh->canceled(res);
      return false;
    }

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeTakeoff(const Takeoff& goal, const GoalHandlePtr& gh, const ResultPtr& res)
{
  // Arm rotors
  if (!armRotors(true)) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "Failed to arm rotors.";
    gh->abort(res);
    return false;
  }

  // TODO: 正常にアームされたかどうかを確認

  // 初期状態を取得
  const auto start_time = now();
  const auto start_pos = odom_->frame.p.clone();
  const auto start_yaw = odom_->frame.M.getYaw();

  // 制約を決定
  const auto max_speed = goal.max_speed > 0. ? goal.max_speed : takeoff_cfg_.max_speed;
  const auto max_accel = goal.max_accel > 0. ? goal.max_accel : takeoff_cfg_.max_accel;
  const auto max_jerk = goal.max_jerk > 0. ? goal.max_jerk : takeoff_cfg_.max_jerk;

  // 軌道を生成
  // TODO: Altitude Frame を考慮
  const traj::TimeOptimalTrajectory traj_z(start_pos.z(), goal.altitude, max_jerk, max_accel, max_speed);

  // 所要時間を取得
  const auto duration = traj_z.duration();
  TOBAS_INFO("Takeoff will take ", duration, " seconds.");

  // 目標状態の固定部分を作成
  kdl::Vector tar_pos(start_pos.x(), start_pos.y(), NAN);
  kdl::Vector tar_vel(0., 0., NAN);

  // 軌道を発行
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // ミッション継続可能かどうかを確認
    if (!checkCurrentStatus(gh, res)) {
      return false;
    }

    // 開始からの経過時間を計算
    const auto cur_time = now();
    const auto dt = (cur_time - start_time).seconds();

    // タイムアウトの確認
    if (goal.timeout > 0. && dt > duration + goal.timeout) {
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::ACCEPTANCE_TIMEOUT;
      res->error_message = "Timed out before reaching the takeoff altitude tolerance.";
      gh->abort(res);
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
    if (gh->is_canceling()) {
      tar_vel.setZero();
    }

    // コマンドを発行
    publishCommands(cur_time, tar_pos, tar_vel, 0., 0., start_yaw);

    // アクション中止の場合は終了
    if (gh->is_canceling()) {
      gh->canceled(res);
      return false;
    }

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeLand(const Land& goal, const GoalHandlePtr& gh, const ResultPtr& res)
{
  // 初期状態を取得
  const auto start_time = now();
  const auto start_pos = odom_->frame.p.clone();
  const kdl::Euler start_rpy(odom_->frame.M);

  // 最新 (直前のコマンド) の目標位置が存在すればそれ，存在しなければ現在位置を目標着陸地点とする．
  double tar_x, tar_y;
  if (last_setpoint_) {
    std::tie(tar_x, tar_y) = tbs::gnssToCartRelative(
      last_setpoint_->latitude, last_setpoint_->longitude, gnss_origin_->latitude, gnss_origin_->longitude);
  }
  else {
    tar_x = start_pos.x();
    tar_y = start_pos.y();
  }

  // 下降速度を決定
  const auto speed = goal.speed > 0. ? goal.speed : land_cfg_.speed;

  // 姿勢の起動を生成
  const auto roll_duration = std::abs(start_rpy.roll) / kAttitudeRate;
  const auto pitch_duration = std::abs(start_rpy.pitch) / kAttitudeRate;
  const traj::LinearSpline traj_roll(start_rpy.roll, 0., roll_duration);
  const traj::LinearSpline traj_pitch(start_rpy.pitch, 0., pitch_duration);

  // 姿勢を戻しながら下降
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // ミッション継続可能かどうかを確認
    if (!checkCurrentStatus(gh, res)) {
      return false;
    }

    // 着陸検知したらモータを停止して終了
    if (landed_->data) {
      TOBAS_INFO("Landing detected. Stopping motors.");
      if (!armRotors(false)) {
        res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
        res->error_message = "Failed to disarm rotors.";
        gh->abort(res);
        return false;
      }
      return true;
    }

    // 現在時刻における目標位置姿勢を計算
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();
    const auto tar_z = start_pos.z() - speed * t;
    const kdl::Vector tar_pos(tar_x, tar_y, tar_z);
    kdl::Vector tar_vel(0., 0., -speed);
    const auto tar_roll = traj_roll.get(t).p;
    const auto tar_pitch = traj_pitch.get(t).p;
    const auto& tar_yaw = start_rpy.yaw;

    // アクション中止の場合は目標速度を0にする
    if (gh->is_canceling()) {
      tar_vel.setZero();
    }

    // コマンドを発行
    publishCommands(cur_time, tar_pos, tar_vel, tar_roll, tar_pitch, tar_yaw);

    // アクション中止の場合は終了
    if (gh->is_canceling()) {
      gh->canceled(res);
      return false;
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
    res->error_message = "Launch point is not set.";
    gh->abort(res);
    return false;
  }

  // Save the latest setpoint
  last_setpoint_ = std::make_shared<GeoPoint>(*launch_point_);

  // ウェイポイントのゴールを作成
  Waypoint wp;
  wp.altitude_frame = AltitudeFrame::kRelativeToHome;
  wp.max_horizontal_velocity = goal.max_horizontal_velocity;
  wp.max_vertical_velocity = goal.max_vertical_velocity;
  wp.max_horizontal_accel = goal.max_horizontal_accel;
  wp.max_vertical_accel = goal.max_vertical_accel;
  wp.max_horizontal_jerk = goal.max_horizontal_jerk;
  wp.max_vertical_jerk = goal.max_vertical_jerk;
  wp.acceptance_radius = goal.acceptance_radius;
  wp.altitude_tolerance = goal.altitude_tolerance;
  wp.timeout = goal.timeout;

  // 目標高度を決定
  const auto& cur_lat = gnss_->latitude;
  const auto& cur_lon = gnss_->longitude;
  const auto& cur_alt = odom_->frame.p.z();
  const auto& tar_lat = launch_point_->latitude;
  const auto& tar_lon = launch_point_->longitude;
  const auto [x_diff, y_diff] = tbs::gnssToCartRelative(cur_lat, cur_lon, tar_lat, tar_lon);
  const auto xy_dist = math::norm(x_diff, y_diff);
  const auto min_alt_goal = goal.min_altitude > 0. ? goal.min_altitude : rtl_cfg_.min_alt;
  const auto min_alt = std::min<double>(min_alt_goal, xy_dist);  // 45度逆円錐ルール
  wp.altitude = std::max(cur_alt, min_alt);

  // 現在の高度がRTLの最低高度よりも低い場合はそこまで上昇
  if (cur_alt < min_alt) {
    wp.latitude = cur_lat;
    wp.longitude = cur_lon;
    wp.auto_heading = false;
    if (!executeWaypoint(wp, gh, res)) {
      return false;
    }
  }

  // アームした地点まで移動
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

void MulticopterMissionExecutorNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  odom_ = odom;
}

void MulticopterMissionExecutorNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  if (!arming_) {
    arming_ = arming;
    return;
  }

  // アームされた座標を保存
  if (!arming_->data && arming->data) {
    if (gnss_ && gnss_->fix_type == tobas_msgs::msg::Gnss::FIX_3D) {
      launch_point_ = std::make_shared<GeoPoint>(gnss_->latitude, gnss_->longitude);
    }
  }

  // ディスアームされたら座標をリセット
  if (arming_->data && !arming->data) {
    launch_point_.reset();
  }

  arming_ = arming;
}

void MulticopterMissionExecutorNode::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  gnss_ = gnss;
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
    TOBAS_WARN("Odometry is not received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!arming_) {
    TOBAS_WARN("Arming status is not received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!gnss_) {
    TOBAS_WARN("GNSS is not received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!gnss_origin_) {
    TOBAS_WARN("GNSS origin is not received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!landed_) {
    TOBAS_WARN("Landed state is not received yet.");
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
        if (!tbs::fromBytes(item.data, waypoint)) {
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
        if (!tbs::fromBytes(item.data, takeoff)) {
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
        if (!tbs::fromBytes(item.data, land)) {
          TOBAS_ERROR("Mission No. ", idx, ": Size mismatch.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        break;
      }
      case kReturnToLaunch: {
        ReturnToLaunch rtl;
        if (!tbs::fromBytes(item.data, rtl)) {
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

  // Reset the old setpoint
  last_setpoint_.reset();

  // Create result
  const auto res = std::make_shared<Result>();

  // Get goal
  const auto goal = gh->get_goal();

  // Execute mission
  for (const auto& [idx, item] : std::views::enumerate(goal->items)) {
    TOBAS_INFO("Start mission No. ", idx);

    // Publish the current mission #
    const auto feedback = std::make_shared<Action::Feedback>();
    feedback->current_index = idx;
    gh->publish_feedback(feedback);

    switch (item.type) {
      case kWaypoint: {
        Waypoint waypoint;
        tbs::fromBytes(item.data, waypoint);
        if (!executeWaypoint(waypoint, gh, res)) {
          is_executing_ = false;
          return;
        }
        break;
      }
      case kTakeoff: {
        Takeoff takeoff;
        tbs::fromBytes(item.data, takeoff);
        if (!executeTakeoff(takeoff, gh, res)) {
          is_executing_ = false;
          return;
        }
        break;
      }
      case kLand: {
        Land land;
        tbs::fromBytes(item.data, land);
        if (!executeLand(land, gh, res)) {
          is_executing_ = false;
          return;
        }
        break;
      }
      case kReturnToLaunch: {
        ReturnToLaunch rtl;
        tbs::fromBytes(item.data, rtl);
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
