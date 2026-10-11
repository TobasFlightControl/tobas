// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/kinematics.hpp>
#include <tobas_kdl/conversion/coordinates.hpp>
#include <tobas_kdl/euler.hpp>
#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/command_priority_handler.hpp>
#include <tobas_tools/coordinates.hpp>
#include <tobas_tools/fixed_wing.hpp>

#include <tobas_command_msgs/msg/aile_elev_rud_throttle.hpp>
#include <tobas_command_msgs/msg/roll_pitch_side_slip_throttle.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_fixed_wing_controller/incomplete_differentiation.hpp"
#include "tobas_fixed_wing_controller/mixer.hpp"

namespace tobas
{
namespace fixed_wing
{
class ControllerNode : public BaseNode
{
  using self = ControllerNode;
  using super = BaseNode;
  static constexpr double kBetaDotCutOffFreq = 30.0;  // beta dot filterのカットオフ周波数

public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Drone drone_;
  kdl::Tree tree_;

  kdl::TreeMassHolder mass_holder_;
  Eigen::VectorXd min_thrusts_;
  Eigen::VectorXd max_thrusts_;
  Eigen::VectorXd min_deflections_;
  Eigen::VectorXd max_deflections_;
  IncompleteDifferentiation beta_dot_filter_;

  // State
  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool topics_received_ = false;
  CommandPriorityHandler cmd_priority_handler_;
  tobas_msgs::msg::FluidPressure::ConstSharedPtr air_pressure_;         // Atmospheric pressure
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_flu_;  // Current state in the FLU coordinate system
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;                      // Rotor arming state

  // Command
  struct RollPitchSideSlip
  {
    double roll;       // [rad]
    double pitch;      // [rad]
    double side_slip;  // [rad]
  };
  std::unique_ptr<RollPitchSideSlip> tar_roll_pitch_sideslip_ = nullptr;
  std::unique_ptr<kdl::Vector> tar_drpb_ = nullptr;
  kdl::Vector tar_dgyro_;
  std::unique_ptr<Eigen::VectorXd> thrusts_ = nullptr;
  std::unique_ptr<Eigen::VectorXd> deflections_ = nullptr;

  // Controller
  Mixer mixer_;
  // double throttle_gain_thresh_;  // [-]
  struct RotationControlParameters
  {
    double atti_wn, head_wn;      // [rad/s]
    double atti_zeta, head_zeta;  // [-]
    // TODO: add integral error gain while airborne
  } rot_ctrl_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> tar_angles_pub_;
  ros2::PublisherPtr<tobas_msgs::OdometryStamped> setpoint_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressure> air_pressure_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::AileElevRudThrottle> manual_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottle> stabilize_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool initialize();
  void setInputLimits();
  void publishThrusts(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& thrusts);
  void publishDeflections(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& deflections);
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& priority);

  // Parameter callbacks
  bool attitudeNaturalFreqCb(const double& p);
  bool attitudeDampingRatioCb(const double& p);
  bool headingNaturalFreqCb(const double& p);
  bool headingDampingRatioCb(const double& p);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& pressure);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom_flu);
  void manualCmdCb(const tobas_command_msgs::msg::AileElevRudThrottle::ConstSharedPtr& cmd);
  void stabilizeCmdCb(const tobas_command_msgs::msg::RollPitchSideSlipThrottle::ConstSharedPtr& cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(node::kController, nodeOptions_DParam(options)), mass_holder_(tree_), mixer_(drone_, tree_)
{
  // setup beta_dot filter
  beta_dot_filter_.setCutoffFrequency(kBetaDotCutOffFreq);

  // Register dynamics parameters
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFreqCb, this, 1.0, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFreqCb, this, 0.2, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 0.1, 10, 1, 20);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 0.1, 10, 1, 20);

  // Register publishers.
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(topic::kRotorThrustsCmd);
  tar_angles_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd);
  setpoint_pub_ = createPublisher<tobas_msgs::OdometryStamped>(topic::kTrajSetpoint);

  // Register subscribers.
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  air_pressure_sub_ = createSubscriber(topic::kAirPressure, &self::airPressureCb, this);
  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
  manual_cmd_sub_ = createSubscriber(topic::kAileElevRudThrottleCmd, &self::manualCmdCb, this);
  stabilize_cmd_sub_ = createSubscriber(topic::kRollPitchSideSlipThrottleCmd, &self::stabilizeCmdCb, this);

  // Register timers.
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::initialize()
{
  if (!mass_holder_.updateInternalDataStructures()) {
    return false;
  }
  if (!mixer_.updateInternalDataStructures()) {
    return false;
  }

  setInputLimits();

  is_initialized_ = true;
  return true;
}

void ControllerNode::setInputLimits()
{
  min_thrusts_.conservativeResize(drone_.prop->numRotors());
  max_thrusts_.conservativeResize(drone_.prop->numRotors());
  min_deflections_.conservativeResize(drone_.fixed_wing->numControlSurfaces());
  max_deflections_.conservativeResize(drone_.fixed_wing->numControlSurfaces());

  for (const auto& [idx, elem] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& link_name = elem.first;
    min_thrusts_(idx) = drone_.prop->minThrust(link_name);
    max_thrusts_(idx) = drone_.prop->maxThrust(link_name);
  }

  size_t cs_idx = 0;
  for (const auto& [_, cs] : drone_.fixed_wing->control_surfaces) {
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();
    min_deflections_(cs_idx) = joint.lower_limit;
    max_deflections_(cs_idx) = joint.upper_limit;
    ++cs_idx;
  }
}

void ControllerNode::publishThrusts(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& thrusts)
{
  assert(static_cast<size_t>(thrusts.size()) == drone_.prop->numRotors());

  auto thrusts_msg = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
  thrusts_msg->header.stamp = stamp;

  for (const auto& [idx, elem] : std::views::enumerate(drone_.prop->rotors)) {
    thrusts_msg->thrusts.emplace_back();
    thrusts_msg->thrusts.back().link_name = elem.first;
    thrusts_msg->thrusts.back().thrust = std::max(thrusts(idx), 0.0);
  }

  tar_thrusts_pub_->publish(std::move(thrusts_msg));
}

void ControllerNode::publishDeflections(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& deflections)
{
  assert(static_cast<size_t>(deflections.size()) == drone_.fixed_wing->numControlSurfaces());

  auto tar_angles_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  tar_angles_msg->header.stamp = stamp;

  for (const auto& [idx, cs_item] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& link_name = cs_item.first;
    const auto& joint = tree_.getSegment(link_name)->second.segment.joint();

    tar_angles_msg->commands.emplace_back();
    tar_angles_msg->commands.back().name = joint.name;
    tar_angles_msg->commands.back().data = deflections(idx);
  }

  tar_angles_pub_->publish(std::move(tar_angles_msg));
}

bool ControllerNode::isCommandAccepted(const tobas_command_msgs::msg::Priority& priority)
{
  if (!topics_received_) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because some topics have not been received yet.");
    return false;
  }

  if (!arming_->data) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because the vehicle is disarmed.");
    return false;
  }

  if (!cmd_priority_handler_.update(priority.data, now())) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return false;
  }

  return true;
}

bool ControllerNode::attitudeNaturalFreqCb(const double& p)
{
  rot_ctrl_.atti_wn = p;
  return true;
}

bool ControllerNode::attitudeDampingRatioCb(const double& p)
{
  rot_ctrl_.atti_zeta = p;
  return true;
}

bool ControllerNode::headingNaturalFreqCb(const double& p)
{
  rot_ctrl_.head_wn = p;
  return true;
}

bool ControllerNode::headingDampingRatioCb(const double& p)
{
  rot_ctrl_.head_zeta = p;
  return true;
}

void ControllerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;

  if (tree_received_) {
    if (!initialize()) {
      TOBAS_FATAL("Error occurred while initializing controller.");
      return;
    }
  }

  drone_received_ = true;
}

void ControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  if (drone_received_) {
    if (!initialize()) {
      TOBAS_FATAL("Error occurred while initializing controller.");
      return;
    }
  }

  tree_received_ = true;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (!arming->data) {
    tar_roll_pitch_sideslip_.reset();
    thrusts_.reset();
    deflections_.reset();
  }
}

void ControllerNode::airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& pressure)
{
  air_pressure_ = pressure;
}

void ControllerNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom_flu)
{
  if (!odom_flu_) {
    odom_flu_ = odom_flu;
    return;
  }

  // Compute elapsed time and update odometry.
  const auto& cur_time = odom_flu->header.stamp;
  const auto dt = (odom_flu->header.stamp - odom_flu_->header.stamp).seconds();
  odom_flu_ = odom_flu;

  // Create the setpoint message.
  auto setpoint = std::make_unique<tobas_msgs::OdometryStamped>();
  setpoint->header.stamp = cur_time;
  setpoint->odom.setNaN();

  // Aliases.
  const auto& cur_rot = odom_flu->odom.odom.frame.M;
  const auto B_T_G =
    kdl::Frame(kdl::Rotation::Identity(), drone_.fixed_wing->vehicle.ref);  // base_link座標系で表したCoG座標系のFrame
  const auto cur_twist_G = B_T_G.inverse() * odom_flu->odom.odom.twist;
  const auto& cur_vel_G = cur_twist_G.vel;
  const auto& cur_gyro_G = cur_twist_G.rot;
  const auto airspeed = cur_vel_G.norm();

  // Attitude controller.
  if (tar_roll_pitch_sideslip_) {
    if (!tar_drpb_) {
      tar_drpb_ = std::make_unique<kdl::Vector>();
    }

    // Determine gains.
    const auto atti_wn = rot_ctrl_.atti_wn;
    const auto head_wn = rot_ctrl_.head_wn;
    const auto atti_angle_gain = atti_wn / rot_ctrl_.atti_zeta / 2;
    const auto head_angle_gain = head_wn / rot_ctrl_.atti_zeta / 2;

    // Determine target attitude.
    // TODO: Limit the target-attitude rate during transition to avoid discontinuous target-attitude changes
    // when switching from attitude-control mode to position-control mode.
    auto tar_rpb = *tar_roll_pitch_sideslip_;

    // yaw角の大きなズレの影響を考慮外にするためにEuler角による引き算を行う
    const kdl::Euler cur_rpy(cur_rot);
    const auto roll_err = algo::wrapPi(tar_rpb.roll - cur_rpy.roll);
    const auto pitch_err = algo::wrapPi(tar_rpb.pitch - cur_rpy.pitch);
    kdl::Vector cur_vel_G_frd;
    kdl::vectorFluToFrd(cur_vel_G, cur_vel_G_frd);
    const auto beta = angleOfSideSlip(cur_vel_G_frd.x(), cur_vel_G_frd.y(), cur_vel_G_frd.z());
    beta_dot_filter_.update(beta, dt);
    const auto beta_err = algo::wrapPi(tar_rpb.side_slip - beta);
    // std::cout << "beta : " << beta / M_PI * 180.0 << " [deg]" << std::endl;
    // std::cout << "beta_err : " << beta_err / M_PI * 180.0 << " [deg]" << std::endl;
    // std::cout << "beta_sp : " << tar_rpb.sideslip / M_PI * 180.0 << " [deg]" << std::endl;

    // TODO: Accumulate integral error while airborne.

    // Compute target Euler angle rates.
    *tar_drpb_ = kdl::Vector(atti_angle_gain * roll_err, atti_angle_gain * pitch_err, head_angle_gain * beta_err);

    // Fill the feedback message.
    setpoint->odom.frame.M =
      kdl::Euler(tar_roll_pitch_sideslip_->roll, tar_roll_pitch_sideslip_->pitch, 0.0).toRotation();
  }

  if (tar_drpb_) {
    // Angular velocity controller.
    {
      // Determine gains.
      const auto atti_wn = rot_ctrl_.atti_wn;
      const auto head_wn = rot_ctrl_.head_wn;
      const auto atti_rate_gain = atti_wn * rot_ctrl_.atti_zeta * 2;
      const auto head_rate_gain = head_wn * rot_ctrl_.head_zeta * 2;
      const kdl::Vector rate_gain(atti_rate_gain, atti_rate_gain, head_rate_gain);

      // Compute target angular acceleration.
      // roll = pitch = 0のときbeta = -yaw_frd = yaw_flu
      // TODO: 若干beta_dotが振動気味
      // std::cout << "tar_drpb_->z() : " << tar_drpb_->z() / M_PI * 180.0 << " [deg/s]" << std::endl;
      // std::cout << "beta_dot_filter_.getValue() : " << beta_dot_filter_.getValue() / M_PI * 180.0 << " [deg/s]" << std::endl;
      tar_dgyro_ = kdl::Vector(
        atti_rate_gain * (tar_drpb_->x() - cur_gyro_G.x()),
        atti_rate_gain * (tar_drpb_->y() - cur_gyro_G.y()),
        head_rate_gain * (tar_drpb_->z() - beta_dot_filter_.getValue()));

      // Fill the feedback message.
      setpoint->odom.twist.rot = *tar_drpb_;
    }

    // Mixer.
    {
      const auto rho = st::pressureToDensity(air_pressure_->pressure);
      if (!mixer_.solve(dt, rho, airspeed, tar_dgyro_)) {
        TOBAS_FATAL("Failed to solve the mixing equation.");
        return;
      }

      // Fill the feedback message.
      setpoint->odom.accel.angular = tar_dgyro_;
    }

    if (!deflections_) {
      deflections_ = std::make_unique<Eigen::VectorXd>();
    }
    *deflections_ = Eigen::VectorXd::Zero(drone_.fixed_wing->numControlSurfaces());
    for (size_t idx = 0; idx < drone_.fixed_wing->numControlSurfaces(); idx++) {
      deflections_->operator()(idx) =
        std::clamp(mixer_.getDeflection(idx), min_deflections_(idx), max_deflections_(idx));
    }
  }

  if (deflections_) {
    publishThrusts(odom_flu->header.stamp, *thrusts_);
    publishDeflections(odom_flu->header.stamp, *deflections_);
  }

  // Publish the feedback message.
  setpoint_pub_->publish(std::move(setpoint));
}

void ControllerNode::manualCmdCb(const tobas_command_msgs::msg::AileElevRudThrottle::ConstSharedPtr& manual_cmd)
{
  if (!isCommandAccepted(manual_cmd->priority)) {
    return;
  }

  // Stop the outer control loop.
  tar_roll_pitch_sideslip_.reset();
  tar_drpb_.reset();

  // Create the command.
  if (!thrusts_) {
    thrusts_ = std::make_unique<Eigen::VectorXd>();
  }
  if (!deflections_) {
    deflections_ = std::make_unique<Eigen::VectorXd>();
  }

  // Set command
  // thrusts
  *thrusts_ = Eigen::VectorXd::Zero(drone_.prop->numRotors());
  for (const auto& [idx, elem] : std::views::enumerate(drone_.prop->rotors)) {
    thrusts_->operator()(idx) = max_thrusts_(idx) * manual_cmd->throttle;
  }
  // control surface deflections
  *deflections_ = Eigen::VectorXd::Zero(drone_.fixed_wing->numControlSurfaces());
  for (const auto& [idx, cs_item] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto type = cs_item.second.type;
    if (type == ControlSurfaceType::kAileron) {
      deflections_->operator()(idx) = math::remap(
        manual_cmd->aileron,
        manual_cmd->MIN_DEFLECTION,
        manual_cmd->MAX_DEFLECTION,
        min_deflections_(idx),
        max_deflections_(idx));
    }
    else if (type == ControlSurfaceType::kElevator) {
      deflections_->operator()(idx) = math::remap(
        manual_cmd->elevator,
        manual_cmd->MIN_DEFLECTION,
        manual_cmd->MAX_DEFLECTION,
        min_deflections_(idx),
        max_deflections_(idx));
    }
    else if (type == ControlSurfaceType::kRudder) {
      deflections_->operator()(idx) = math::remap(
        manual_cmd->rudder,
        manual_cmd->MIN_DEFLECTION,
        manual_cmd->MAX_DEFLECTION,
        min_deflections_(idx),
        max_deflections_(idx));
    }
    else {
      deflections_->operator()(idx) = 0.0;
    }
  }
}

void ControllerNode::stabilizeCmdCb(const tobas_command_msgs::msg::RollPitchSideSlipThrottle::ConstSharedPtr& cmd)
{
  if (!isCommandAccepted(cmd->priority)) {
    return;
  }

  // Check command range.
  if (std::abs(cmd->roll) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (std::abs(cmd->pitch) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }

  // TODO: Stop the outer control loop.

  // Create the command.
  if (!tar_roll_pitch_sideslip_) {
    tar_roll_pitch_sideslip_ = std::make_unique<RollPitchSideSlip>();
  }
  if (!thrusts_) {
    thrusts_ = std::make_unique<Eigen::VectorXd>();
  }

  // Update the command.
  tar_roll_pitch_sideslip_->roll = cmd->roll;
  tar_roll_pitch_sideslip_->pitch = cmd->pitch;
  tar_roll_pitch_sideslip_->side_slip = cmd->side_slip;
  // thrusts
  *thrusts_ = Eigen::VectorXd::Zero(drone_.prop->numRotors());
  for (const auto& [idx, elem] : std::views::enumerate(drone_.prop->rotors)) {
    thrusts_->operator()(idx) = max_thrusts_(idx) * cmd->throttle;
  }
}

void ControllerNode::checkTopicsTimerCb()
{
  if (!drone_received_) {
    TOBAS_WARN("Waiting for \"", topic::kDrone, "\".");
    return;
  }

  if (!tree_received_) {
    TOBAS_WARN("Waiting for \"", topic::kKdlTree, "\".");
    return;
  }

  if (!air_pressure_) {
    TOBAS_WARN("Waiting for \"", topic::kAirPressure, "\".");
    return;
  }

  if (!odom_flu_) {
    TOBAS_WARN("Waiting for \"", topic::kOdometry, "\".");
    return;
  }

  topics_received_ = true;
  check_topics_timer_->cancel();
}
}  // namespace fixed_wing
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::fixed_wing::ControllerNode)
