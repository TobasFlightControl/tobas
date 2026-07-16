// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <span>
#include <vector>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_math/linalg.hpp>
#include <tobas_mission_items/mission.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/byte.hpp>
#include <tobas_std_tools/gnss.hpp>
#include <tobas_trajectory_generation/offline/catmull_rom.hpp>
#include <tobas_trajectory_generation/offline/jerk_limited.hpp>
#include <tobas_trajectory_generation/offline/linear.hpp>

#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>
#include <tobas_mission_msgs/action/execute_mission.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/geodetic_coordinates.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
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
namespace
{
using CatmullRomPath = traj::CatmullRomPath<Eigen::Vector3d>;

/* Return true when `value` is finite and positive. */
inline bool isPositive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

inline double selectConservativeLimit(double current, double candidate)
{
  if (!isPositive(candidate)) {
    return current;
  }
  return isPositive(current) ? std::min(current, candidate) : candidate;
}

struct PathComponentScale
{
  // Maximum horizontal and vertical component coefficients for path arc length `s`.
  // For example, `horizontal = 1` and `vertical = 0` is a horizontal-only path.
  // `horizontal = 0` and `vertical = 1` is a vertical-only path.
  // `horizontal` and `vertical` are separate norms, so their sum is not necessarily 1.
  double horizontal = 0.0;
  double vertical = 0.0;
};

size_t selectPathConstraintSampleCount(const CatmullRomPath& path)
{
  constexpr double kPathConstraintSampleInterval = 1.0;  // [m]
  constexpr size_t kPathConstraintSamplesPerSegment = 20;
  constexpr size_t kMaxPathConstraintSamples = 5000;

  // Increase the sample count by distance for long paths.
  // For short paths with many turns, increase it by segment count.
  const auto length_based_samples = static_cast<size_t>(std::ceil(path.length() / kPathConstraintSampleInterval));
  const auto segment_based_samples = path.segmentCount() * kPathConstraintSamplesPerSegment;
  return std::clamp(std::max(length_based_samples, segment_based_samples), 1UL, kMaxPathConstraintSamples);
}

PathComponentScale getMaxPathComponentScale(const CatmullRomPath& path)
{
  PathComponentScale scale;
  const auto path_length = path.length();
  const auto sample_count = selectPathConstraintSampleCount(path);

  // In a Catmull-Rom curve, the tangent direction changes within each interval.
  // Sample the whole path and use the strictest component coefficients.
  for (size_t sample = 0; sample <= sample_count; ++sample) {
    const auto s = path_length * static_cast<double>(sample) / static_cast<double>(sample_count);
    const auto tangent = path.get(s).tangent;
    scale.horizontal = std::max(scale.horizontal, math::norm(tangent.x(), tangent.y()));
    scale.vertical = std::max(scale.vertical, std::abs(tangent.z()));
  }
  return scale;
}

double selectPathConstraintLimit(double horizontal_limit, double vertical_limit, const PathComponentScale& scale)
{
  // Convert per-axis constraints into scalar constraints along path arc length `s`.
  // Since `v_xy = horizontal_scale * s_dot` and `v_z = vertical_scale * s_dot`,
  // divide each axis limit by the corresponding component coefficient.
  auto path_limit = std::numeric_limits<double>::infinity();
  if (scale.horizontal > 0.0) {
    path_limit = std::min(path_limit, horizontal_limit / scale.horizontal);
  }
  if (scale.vertical > 0.0) {
    path_limit = std::min(path_limit, vertical_limit / scale.vertical);
  }
  return std::isfinite(path_limit) ? path_limit : std::min(horizontal_limit, vertical_limit);
}
}  // namespace

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

  static constexpr double kCommandRate = 100.0;      // [Hz]
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
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;
  tobas_msgs::msg::VehicleHealth::ConstSharedPtr health_;
  tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr gnss_origin_;

  ros2::PublisherPtr<tobas_command_msgs::Angle> angle_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAcc> pva_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccYaw> pvay_pub_;
  ros2::PublisherPtr<tobas_command_msgs::PosVelAccPitchYaw> pvapy_pub_;

  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryStamped> setpoint_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::GeodeticCoordinates> gnss_origin_sub_;
  ros2::SubscriberPtr<tobas_msgs::RCInput> rcin_sub_;

  ros2::SyncServiceClient<tobas_msgs::srv::SetArm>::SharedPtr set_arm_sc_;

  ros2::ActionServerPtr<Action> as_;

  void getStaticRosParams();

  /* Initialize the command from the setpoint if it exists, otherwise from the current state. */
  void initializeCommand();

  /* Publish the command. */
  void publishCommand(const rclcpp::Time& stamp);

  /* Arm or disarm rotors synchronously. */
  bool armRotors(bool arming);

  /* Smoothly stop from the current command. */
  void brake();

  /**
   * @brief Handle shutdown appropriately in response to external requests.
   * This includes cancellation, supersession, and similar requests.
   *
   * @return bool Whether the mission can continue.
   */
  bool handleExternalRequest(const GoalHandlePtr& gh, const ResultPtr& res);

  bool executeWaypoints(std::span<const Waypoint> goals, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeTakeoff(const Takeoff& goal, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeLand(const Land& goal, const GoalHandlePtr& gh, const ResultPtr& res);
  bool executeRTL(const ReturnToLaunch& goal, const GoalHandlePtr& gh, const ResultPtr& res);

  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void setpointCb(const tobas_msgs::OdometryStamped::ConstSharedPtr& setpoint);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
  void gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin);
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
  landed_sub_ = createSubscriber(topic::kLanded, &self::landedCb, this);
  health_sub_ = createSubscriber(topic::kVehicleHealth, &self::healthCb, this);
  gnss_origin_sub_ = createSubscriber(topic::kGnssOrigin, &self::gnssOriginCb, this, true, true);
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

  // Determine the initial command from the setpoint or current state.
  // Do not use the acceleration estimate because it may mostly contain vibration components.
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
      command_.acc.setZero();
    }
  }
  else {
    command_.pos = odom.frame.p;
    odom.frame.M.getRPY(command_.rot.roll, command_.rot.pitch, command_.rot.yaw);
    command_.vel = odom.twist.vel;
    command_.acc.setZero();
  }
}

void MulticopterMissionExecutorNode::publishCommand(const rclcpp::Time& stamp)
{
  // Set the command priority according to the mission priority.
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
  const auto req = std::make_shared<tobas_msgs::srv::SetArm::Request>();
  req->arming = arming;

  const auto res = set_arm_sc_->sendRequestAndWait(req);
  if (!res) {
    TOBAS_ERROR("Failed to call \"", service::kSetArm, "\" service.");
    return false;
  }

  if (!res->success) {
    TOBAS_ERROR("Failed to set the arming status: ", res->message);
    return false;
  }

  return true;
}

void MulticopterMissionExecutorNode::brake()
{
  // Generate trajectories.
  const Eigen::Vector2d pxy0(command_.pos.x(), command_.pos.y());
  const Eigen::Vector2d vxy0(command_.vel.x(), command_.vel.y());
  const Eigen::Vector2d axy0(command_.acc.x(), command_.acc.y());
  const auto vxy0_norm = vxy0.norm();
  const auto axy0_norm = axy0.norm();
  const auto dir_xy = vxy0_norm > 0.0 ? (vxy0 / vxy0_norm).eval() : Eigen::Vector2d::Zero();
  const StopTrajectory traj_xy(
    0.0, vxy0_norm, axy0_norm * math::sign(vxy0.dot(axy0)), wp_cfg_.max_hor_acc, wp_cfg_.max_hor_jerk);

  const auto pz0 = command_.pos.z();  // Must be copy.
  const auto vz0 = command_.vel.z();  // Must be copy.
  const auto az0 = command_.acc.z();  // Must be copy.
  const auto vz0_norm = std::abs(vz0);
  const auto az0_norm = std::abs(az0);
  const auto dir_z = math::sign(vz0);
  const StopTrajectory traj_z(
    0.0, vz0_norm, az0_norm * math::sign(vz0 * az0), wp_cfg_.max_ver_acc, wp_cfg_.max_ver_jerk);

  // Get the duration.
  const auto duration = std::max(traj_xy.duration(), traj_z.duration());
  if (duration < kMinBrakeDuration) {
    return;  // Exit without publishing commands when already almost stopped.
  }
  TOBAS_INFO("The vehicle will stop in ", duration, " seconds.");

  // Publish trajectory commands.
  const auto start_time = now();
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // Calculate the elapsed time from the start.
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();

    // Exit after all commands have been published.
    if (t > duration) {
      return;
    }

    // Get the target state at the current time.
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

    // Publish the command.
    publishCommand(cur_time);

    rate.sleep();
  }
}

bool MulticopterMissionExecutorNode::handleExternalRequest(const GoalHandlePtr& gh, const ResultPtr& res)
{
  // Smoothly stop before exiting when the action is canceled.
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

bool MulticopterMissionExecutorNode::executeWaypoints(
  std::span<const Waypoint> goals,
  const GoalHandlePtr& gh,
  const ResultPtr& res)
{
  if (goals.empty()) {
    return true;
  }

  if (!arming_->data) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "The waypoint mission cannot be started because the vehicle is disarmed.";
    gh->abort(res);
    return false;
  }

  // Waypoints are grouped into one continuous path until stop_at_waypoint requests an arrival check.
  brake();

  const auto start_pos = command_.pos.clone();
  const auto start_rot = command_.rot.clone();

  std::vector<Eigen::Vector3d> path_points;
  path_points.reserve(goals.size() + 1);
  path_points.push_back(start_pos.data);

  for (const auto& goal : goals) {
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
      case kMeanSeaLevel:
        goal_pos.z(goal.altitude - gnss_origin_->altitude);
        break;
      default:
        res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
        res->error_message = "Invalid altitude frame type: " + std::to_string(goal.altitude_frame);
        gh->abort(res);
        return false;
    }
    path_points.push_back(goal_pos.data);
  }

  const CatmullRomPath path(std::move(path_points));
  const auto path_length = path.length();
  if (path_length == 0.0) {
    return true;
  }

  auto max_hor_vel = wp_cfg_.max_hor_vel;
  auto max_hor_acc = wp_cfg_.max_hor_acc;
  auto max_hor_jerk = wp_cfg_.max_hor_jerk;
  auto max_ver_vel = wp_cfg_.max_ver_vel;
  auto max_ver_acc = wp_cfg_.max_ver_acc;
  auto max_ver_jerk = wp_cfg_.max_ver_jerk;
  auto max_head_rate = wp_cfg_.max_head_rate;
  for (const auto& goal : goals) {
    max_hor_vel = selectConservativeLimit(max_hor_vel, goal.max_horizontal_velocity);
    max_hor_acc = selectConservativeLimit(max_hor_acc, goal.max_horizontal_accel);
    max_hor_jerk = selectConservativeLimit(max_hor_jerk, goal.max_horizontal_jerk);
    max_ver_vel = selectConservativeLimit(max_ver_vel, goal.max_vertical_velocity);
    max_ver_acc = selectConservativeLimit(max_ver_acc, goal.max_vertical_accel);
    max_ver_jerk = selectConservativeLimit(max_ver_jerk, goal.max_vertical_jerk);
    max_head_rate = selectConservativeLimit(max_head_rate, goal.max_heading_rate);
  }

  // A 3D path advances along one scalar trajectory `s(t)`.
  // Convert xy/z constraints to limits along `s`, then use the minimum.
  const auto path_component_scale = getMaxPathComponentScale(path);
  const auto max_path_vel = selectPathConstraintLimit(max_hor_vel, max_ver_vel, path_component_scale);
  const auto max_path_acc = selectPathConstraintLimit(max_hor_acc, max_ver_acc, path_component_scale);
  const auto max_path_jerk = selectPathConstraintLimit(max_hor_jerk, max_ver_jerk, path_component_scale);

  const traj::JerkLimitedTrajectory traj_path(0.0, path_length, max_path_jerk, max_path_acc, max_path_vel);

  const auto roll_duration = std::abs(start_rot.roll) / kAttitudeRate;
  const traj::LinearSpline traj_roll(start_rot.roll, 0.0, roll_duration);

  const auto pitch_duration = std::abs(start_rot.pitch) / kAttitudeRate;
  const traj::LinearSpline traj_pitch(start_rot.pitch, 0.0, pitch_duration);

  const auto duration = algo::max(traj_path.duration(), traj_roll.duration(), traj_pitch.duration());
  TOBAS_INFO("Moving along the waypoint path will take ", duration, " seconds.");

  const auto auto_heading = std::ranges::any_of(goals, [](const auto& goal) { return goal.auto_heading; });
  const auto& final_goal = goals.back();
  const auto final_goal_pos = path.get(path_length).pos;

  const auto start_time = now();
  auto prev_time = start_time;
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    if (!handleExternalRequest(gh, res)) {
      return false;
    }

    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();
    const auto dt = (cur_time - prev_time).seconds();
    prev_time = cur_time;

    if (final_goal.timeout > 0.0 && t > duration + final_goal.timeout) {
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::ACCEPTANCE_TIMEOUT;
      res->error_message = "Timed out before reaching the waypoint acceptance radius.";
      gh->abort(res);
      return false;
    }

    const auto& cur_pos = odom_->odom.odom.frame.p;
    if (t > duration) {
      const auto pos_err = final_goal_pos - cur_pos;
      const auto xy_err_abs = math::norm(pos_err.x(), pos_err.y());
      const auto z_err_abs = std::abs(pos_err.z());
      const auto hor_ok = final_goal.acceptance_radius <= 0.0 || xy_err_abs < final_goal.acceptance_radius;
      const auto ver_ok = final_goal.altitude_tolerance <= 0.0 || z_err_abs < final_goal.altitude_tolerance;
      if (hor_ok && ver_ok) {
        return true;
      }
    }

    // `traj_point.p`, `traj_point.v`, and `traj_point.a` are states along path arc length `s`.
    // Map them to 3D position, tangent, and curvature with `CatmullRomPath`.
    const auto traj_point = traj_path.get(t);
    const auto path_point = path.get(traj_point.p);
    command_.pos.data = path_point.pos;
    command_.vel.data = path_point.tangent * traj_point.v;
    command_.acc.data = path_point.tangent * traj_point.a + path_point.curvature * math::sqr(traj_point.v);

    auto yaw = command_.rot.yaw;
    if (auto_heading) {
      const auto tangent_xy_norm = math::norm(path_point.tangent.x(), path_point.tangent.y());
      if (tangent_xy_norm > 0.0) {
        const auto desired_yaw = std::atan2(path_point.tangent.y(), path_point.tangent.x());
        const auto yaw_diff = algo::wrapPi(desired_yaw - yaw);
        const auto max_yaw_step = max_head_rate * dt;
        yaw = algo::wrapPi(yaw + std::clamp(yaw_diff, -max_yaw_step, max_yaw_step));
      }
    }
    command_.rot.set(traj_roll.get(t).p, traj_pitch.get(t).p, yaw);

    publishCommand(cur_time);

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeTakeoff(const Takeoff& goal, const GoalHandlePtr& gh, const ResultPtr& res)
{
  // Verify that the vehicle is disarmed.
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

  // Get the initial target state.
  const auto start_pos = command_.pos.clone();
  const auto start_yaw = command_.rot.yaw;

  // Determine the target altitude.
  double tar_z;  // wrt. the odometry frame
  switch (goal.altitude_frame) {
    case kRelativeToLaunch:
      tar_z = start_pos.z() + goal.altitude;
      break;
    case kMeanSeaLevel:
      tar_z = goal.altitude - gnss_origin_->altitude;
      break;
    default:
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
      res->error_message = "Invalid altitude frame type: " + std::to_string(goal.altitude_frame);
      gh->abort(res);
      return false;
  }

  // Determine constraints.
  const auto max_speed = isPositive(goal.max_speed) ? goal.max_speed : takeoff_cfg_.max_speed;
  const auto max_accel = isPositive(goal.max_accel) ? goal.max_accel : takeoff_cfg_.max_accel;
  const auto max_jerk = isPositive(goal.max_jerk) ? goal.max_jerk : takeoff_cfg_.max_jerk;

  // Generate the trajectory.
  const traj::JerkLimitedTrajectory traj_z(start_pos.z(), tar_z, max_jerk, max_accel, max_speed);

  // Get the duration.
  const auto duration = traj_z.duration();
  TOBAS_INFO("Takeoff will take ", duration, " seconds.");

  // Publish trajectory commands.
  const auto start_time = now();
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // Check whether the mission can continue.
    if (!handleExternalRequest(gh, res)) {
      return false;
    }

    // Calculate the elapsed time from the start.
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();

    // Check for timeout.
    if (goal.timeout > 0.0 && t > duration + goal.timeout) {
      res->error_code.data = tobas_mission_msgs::msg::ErrorCode::ACCEPTANCE_TIMEOUT;
      res->error_message = "Timed out before reaching the takeoff altitude tolerance.";
      gh->abort(res);
      return false;
    }

    // Succeed after all commands have been published and the vehicle is within tolerance.
    const auto& cur_pos = odom_->odom.odom.frame.p;
    const auto alt_err_abs = std::abs(tar_z - cur_pos.z());
    if (t > duration) {
      if (goal.altitude_tolerance <= 0.0 || alt_err_abs < goal.altitude_tolerance) {
        return true;
      }
    }

    // Create the command.
    const auto traj_point_z = traj_z.get(t);
    command_.pos.set(start_pos.x(), start_pos.y(), traj_point_z.p);
    command_.vel.set(0.0, 0.0, traj_point_z.v);
    command_.acc.set(0.0, 0.0, traj_point_z.a);
    command_.rot.set(0.0, 0.0, start_yaw);

    // Publish the command.
    publishCommand(cur_time);

    rate.sleep();
  }

  return false;
}

bool MulticopterMissionExecutorNode::executeLand(const Land& goal, const GoalHandlePtr& gh, const ResultPtr& res)
{
  // Verify that the vehicle is armed.
  if (!arming_->data) {
    res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
    res->error_message = "The land mission cannot be started because the vehicle is disarmed.";
    gh->abort(res);
    return false;
  }

  // Pause before starting.
  brake();

  // Get the initial target state.
  const auto start_pos = command_.pos.clone();
  const auto start_rot = command_.rot.clone();

  // Determine the descent speed.
  const auto speed = isPositive(goal.speed) ? goal.speed : land_cfg_.speed;

  // Generate attitude trajectories.
  const auto roll_duration = std::abs(start_rot.roll) / kAttitudeRate;
  const auto pitch_duration = std::abs(start_rot.pitch) / kAttitudeRate;
  const traj::LinearSpline traj_roll(start_rot.roll, 0.0, roll_duration);
  const traj::LinearSpline traj_pitch(start_rot.pitch, 0.0, pitch_duration);

  // Create objects used for landing detection.
  const auto stop_speed_thresh = std::min<double>(speed / 2, 0.2);
  auto t_last_high_speed = odom_->header.stamp;

  // Descend while restoring attitude.
  const auto start_time = now();
  rclcpp::Rate rate(kCommandRate, get_clock());
  while (rclcpp::ok()) {
    // Check whether the mission can continue.
    if (!handleExternalRequest(gh, res)) {
      return false;
    }

    // Calculate the target pose at the current time.
    const auto cur_time = now();
    const auto t = (cur_time - start_time).seconds();
    const auto tar_z = start_pos.z() - speed * t;
    command_.pos.set(start_pos.x(), start_pos.y(), tar_z);
    command_.vel.set(0.0, 0.0, -speed);
    command_.acc.setZero();
    command_.rot.set(traj_roll.get(t).p, traj_pitch.get(t).p, start_rot.yaw);

    // Publish the command.
    publishCommand(cur_time);

    // Get the latest IMU timestamp.
    const auto imu_time = odom_->header.stamp;  // Copy

    // Calculate the vertical velocity.
    const auto cur_vel_W = odom_->odom.odom.frame.M * odom_->odom.odom.twist.vel;
    const auto& cur_vz = cur_vel_W.z();

    // Calculate the elapsed time since the last high-speed detection.
    if (std::abs(cur_vz) > stop_speed_thresh) {
      t_last_high_speed = imu_time;
    }
    const auto time_from_last_high_speed = imu_time - t_last_high_speed;

    // Calculate the altitude error.
    const auto z_error = tar_z - odom_->odom.odom.frame.p.z();

    // Stop the motors and exit when any of the following conditions is met.
    // 1. Ground reaction force close to the vehicle weight is detected by the common landing detection algorithm.
    // 2. The absolute vertical velocity remains small for a while: https://ardupilot.org/copter/docs/land-mode.html
    // 3. The target altitude differs greatly from the estimated altitude as a last resort.
    if (landed_->landed || time_from_last_high_speed > 1s || z_error < -10.0) {
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

  // Pause before starting.
  brake();

  // Create a waypoint goal.
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

  // Determine the target altitude.
  const auto& cur_pos = odom_->odom.odom.frame.p;
  const auto cur_alt = cur_pos.z() - launch_point_->z();
  const auto xy_dist = math::norm(launch_point_->x() - cur_pos.x(), launch_point_->y() - cur_pos.y());
  const auto min_alt_goal = goal.min_altitude > 0.0 ? goal.min_altitude : rtl_cfg_.min_alt;
  const auto min_alt = std::min<double>(min_alt_goal, xy_dist);  // 45-degree inverted cone rule.
  wp.altitude = std::max(cur_alt, min_alt);

  // Climb to the RTL minimum altitude if the current altitude is lower.
  if (cur_alt < min_alt) {
    const auto [tar_lat, tar_lon] =
      st::cartToGnssRelative(cur_pos.x(), cur_pos.y(), gnss_origin_->latitude, gnss_origin_->longitude);
    wp.latitude = tar_lat;
    wp.longitude = tar_lon;
    wp.auto_heading = false;
    if (!executeWaypoints(std::span<const Waypoint>(&wp, 1), gh, res)) {
      return false;
    }
  }

  // Move to the arming point.
  const auto [tar_lat, tar_lon] =
    st::cartToGnssRelative(launch_point_->x(), launch_point_->y(), gnss_origin_->latitude, gnss_origin_->longitude);
  wp.latitude = tar_lat;
  wp.longitude = tar_lon;
  wp.auto_heading = true;
  if (!executeWaypoints(std::span<const Waypoint>(&wp, 1), gh, res)) {
    return false;
  }

  // Land.
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
  // Initialize.
  if (!arming_) {
    arming_ = arming;
    set_arm_sc_ =
      std::make_shared<ros2::SyncServiceClient<tobas_msgs::srv::SetArm>>(shared_from_this(), service::kSetArm);
    return;
  }

  // Save the arming point.
  if (!arming_->data && arming->data) {
    if (odom_) {
      launch_point_ = std::make_unique<kdl::Vector>(odom_->odom.odom.frame.p);
    }
  }

  // Reset the arming point and setpoint when disarmed.
  if (arming_->data && !arming->data) {
    launch_point_.reset();
    setpoint_.reset();
  }

  arming_ = arming;
}

void MulticopterMissionExecutorNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}

void MulticopterMissionExecutorNode::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  health_ = health;
}

void MulticopterMissionExecutorNode::gnssOriginCb(const tobas_msgs::msg::GeodeticCoordinates::ConstSharedPtr& gnss_origin)
{
  gnss_origin_ = gnss_origin;
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
  TOBAS_INFO("A new mission has been uploaded.");

  // Check mission priority.
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

  // Check the essential topics.
  if (!odom_) {
    TOBAS_WARN("Odometry has not been received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!arming_) {
    TOBAS_WARN("Arming status has not been received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!landed_) {
    TOBAS_WARN("Landed state has not been received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!health_) {
    TOBAS_WARN("Vehicle health has not been received yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Reject the mission if manual control is enabled.
  if (is_manual_ctrl_enabled_) {
    TOBAS_WARN("Mission cannot be executed while manual control is enabled.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check the mission items.
  auto armed = arming_->data;
  for (const auto& [idx, item] : std::views::enumerate(goal->mission.items)) {
    const auto cmd_number = idx + 1;

    switch (item.type) {
      case kWaypoint: {
        Waypoint waypoint;
        if (!st::fromBytes(item.data, waypoint)) {
          TOBAS_WARN("Mission No. ", cmd_number, ": Size mismatch.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (waypoint.latitude < -90 || 90 < waypoint.latitude) {
          TOBAS_WARN("Mission No. ", cmd_number, ": Invalid target latitude.");
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (waypoint.longitude < -180 || 180 < waypoint.longitude) {
          TOBAS_WARN("Mission No. ", cmd_number, ": Invalid target longitude.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (!armed) {
          TOBAS_WARN("Mission No. ", cmd_number, ": The vehicle must be armed before a \"Waypoint\" command.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (!gnss_origin_) {
          TOBAS_WARN("Mission No. ", cmd_number, ": GNSS must be fixed before a \"Waypoint\" command.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        break;
      }
      case kTakeoff: {
        Takeoff takeoff;
        if (!st::fromBytes(item.data, takeoff)) {
          TOBAS_WARN("Mission No. ", cmd_number, ": Size mismatch.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (takeoff.altitude <= 0.0) {
          TOBAS_WARN("Mission No. ", cmd_number, ": Target altitude must be positive.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (armed) {
          TOBAS_WARN("Mission No. ", cmd_number, ": The vehicle must be disarmed before a \"Takeoff\" command.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (!gnss_origin_) {
          TOBAS_WARN("Mission No. ", cmd_number, ": The vehicle cannot takeoff because GNSS has not fixed yet.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (!health_->ok) {
          TOBAS_WARN("Mission No. ", cmd_number, ": The vehicle cannot takeoff because the pre-arm check failed.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        armed = true;

        break;
      }
      case kLand: {
        Land land;
        if (!st::fromBytes(item.data, land)) {
          TOBAS_WARN("Mission No. ", cmd_number, ": Size mismatch.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (!armed) {
          TOBAS_WARN("Mission No. ", cmd_number, ": The vehicle must be armed before a \"Land\" command.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        armed = false;

        break;
      }
      case kReturnToLaunch: {
        ReturnToLaunch rtl;
        if (!st::fromBytes(item.data, rtl)) {
          TOBAS_WARN("Mission No. ", cmd_number, ": Size mismatch.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (!armed) {
          TOBAS_WARN("Mission No. ", cmd_number, ": The vehicle must be armed before a \"RTL\" command.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        if (!gnss_origin_) {
          TOBAS_WARN("Mission No. ", cmd_number, ": GNSS must be fixed before a \"RTL\" command.");
          return rclcpp_action::GoalResponse::REJECT;
        }

        armed = false;

        break;
      }
      default: {
        TOBAS_ERROR("Invalid mission type: ", (int)item.type);
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
  }

  // Update the mission priority.
  mission_priority_ = new_priority;

  // Request interruption if an old mission is running.
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
  // Wait until the previous mission is finished.
  rclcpp::Rate rate(kCommandRate);
  while (rclcpp::ok() && is_executing_) {
    rate.sleep();
  }

  // Now the new mission is in execution.
  is_executing_ = true;
  status_ = kNoProblem;

  // Initialize the command.
  initializeCommand();

  // Create result.
  const auto res = std::make_shared<Result>();

  // Get goal.
  const auto goal = gh->get_goal();

  // Execute mission.
  const auto& items = goal->mission.items;
  for (size_t idx = 0; idx < items.size();) {
    const auto& item = items[idx];
    const auto cmd_number = idx + 1;
    TOBAS_INFO("Start mission No. ", cmd_number);

    // Publish the current mission number.
    const auto feedback = std::make_shared<Action::Feedback>();
    feedback->current_command_index = idx;
    gh->publish_feedback(feedback);

    res->last_command_index = idx;

    switch (item.type) {
      case kWaypoint: {
        std::vector<Waypoint> waypoints;
        for (; idx < items.size() && items[idx].type == kWaypoint; ++idx) {
          Waypoint waypoint;
          st::fromBytes(items[idx].data, waypoint);
          waypoints.push_back(waypoint);
          if (waypoint.stop_at_waypoint) {
            ++idx;
            break;
          }
        }
        res->last_command_index = idx - 1;
        if (!executeWaypoints(std::span<const Waypoint>(waypoints.data(), waypoints.size()), gh, res)) {
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
        ++idx;
        break;
      }
      case kLand: {
        Land land;
        st::fromBytes(item.data, land);
        if (!executeLand(land, gh, res)) {
          is_executing_ = false;
          return;
        }
        ++idx;
        break;
      }
      case kReturnToLaunch: {
        ReturnToLaunch rtl;
        st::fromBytes(item.data, rtl);
        if (!executeRTL(rtl, gh, res)) {
          is_executing_ = false;
          return;
        }
        ++idx;
        break;
      }
      default: {
        res->error_code.data = tobas_mission_msgs::msg::ErrorCode::OTHER_ERROR;
        res->error_message = "Invalid mission type: " + std::to_string(item.type);
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
