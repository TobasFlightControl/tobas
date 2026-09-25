// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_control/lqd.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/kinematics.hpp>
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
#include <tobas_command_msgs_adapter/angle_throttle.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_fixed_wing_controller/mixer.hpp"


namespace tobas
{
namespace fixed_wing
{
class ControllerNode : public BaseNode
{
  using self = ControllerNode;
  using super = BaseNode;

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

  // State
  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool topics_received_ = false;
  CommandPriorityHandler cmd_priority_handler_;
  tobas_msgs::msg::FluidPressure::ConstSharedPtr air_pressure_;           // Atmospheric pressure
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_flu_;    // Current state in the FLU coordinate system
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;                        // Rotor arming state

  // Command
  std::unique_ptr<kdl::Euler> tar_angle_ = nullptr;
  std::unique_ptr<kdl::Vector> tar_gyro_ = nullptr;
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
  ros2::SubscriberPtr<tobas_command_msgs::AngleThrottle> stabilize_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool initialize();
  void setInputLimits();
  void publishThrusts(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& thrusts);
  void publishDeflections(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& deflections);
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& priority);
  kdl::Vector computeEulerError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy);

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
  void angleCmdCb(const tobas_command_msgs::AngleThrottle::ConstSharedPtr& cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(node::kController, nodeOptions_DParam(options)), mass_holder_(tree_), mixer_(drone_, tree_)
{
  // Register dynamics parameters
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFreqCb, this, 1.0, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFreqCb, this, 0.5, 10, 1, 30, " rad/s");
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
  stabilize_cmd_sub_ = createSubscriber(topic::kAngleThrotCmd, &self::angleCmdCb, this);

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

kdl::Vector ControllerNode::computeEulerError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy)
{
  // Note that a straight line connecting two Euler angles is not the shortest distance in rotation.
  const auto roll_err = algo::wrapPi(tar_rpy.roll - cur_rpy.roll);
  const auto pitch_err = algo::wrapPi(tar_rpy.pitch - cur_rpy.pitch);
  const auto yaw_err = algo::wrapPi(tar_rpy.yaw - cur_rpy.yaw);
  return { roll_err, pitch_err, yaw_err };
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
    tar_angle_.reset();
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
  const auto& cur_vel_B = odom_flu->odom.odom.twist.vel;
  const auto& cur_gyro_B = odom_flu->odom.odom.twist.rot;

  // Attitude controller.
  if (tar_angle_) {
    if (!tar_gyro_) {
      tar_gyro_ = std::make_unique<kdl::Vector>();
    }

    // Determine gains.
    const auto atti_wn = rot_ctrl_.atti_wn;
    const auto head_wn = rot_ctrl_.head_wn;
    const auto atti_angle_gain = atti_wn / rot_ctrl_.atti_zeta / 2;
    const auto head_angle_gain = head_wn / rot_ctrl_.head_zeta / 2;
    const kdl::Vector angle_gain(atti_angle_gain, atti_angle_gain, head_angle_gain);

    // Determine target attitude.
    // TODO: Limit the target-attitude rate during transition to avoid discontinuous target-attitude changes
    // when switching from attitude-control mode to position-control mode.
    auto tar_rpy = *tar_angle_;

    // yaw角の大きなズレの影響を考慮外にするためにEuler角による引き算を行う
    const kdl::Euler cur_rpy(cur_rot);
    const auto ep = computeEulerError(cur_rpy, tar_rpy);

    // TODO: Accumulate integral error while airborne.

    // Compute target Euler angle rates.
    const auto tar_drpy = angle_gain.hadamard(ep);

    // Convert Euler angle rates to gyro values.
    *tar_gyro_ = eigen::angvelFromEulerrateLocal(tar_drpy.data, cur_rpy.roll, cur_rpy.pitch);

    // Fill the feedback message.
    setpoint->odom.frame.M = tar_rpy.toRotation();
  }

  if (tar_gyro_) {
    // Angular velocity controller.
    {
      // Determine gains.
      const auto atti_wn = rot_ctrl_.atti_wn;
      const auto head_wn = rot_ctrl_.head_wn;
      const auto atti_rate_gain = atti_wn * rot_ctrl_.atti_zeta * 2;
      const auto head_rate_gain = head_wn * rot_ctrl_.head_zeta * 2;
      const kdl::Vector rate_gain(atti_rate_gain, atti_rate_gain, head_rate_gain);

      // Compute target angular acceleration.
      tar_dgyro_ = rate_gain.hadamard(*tar_gyro_ - cur_gyro_B);

      // Fill the feedback message.
      setpoint->odom.twist.rot = *tar_gyro_;
    }

    // Mixer.
    {
      const auto rho = st::pressureToDensity(air_pressure_->pressure);
      if (!mixer_.solve(dt, rho, cur_vel_B, cur_gyro_B, tar_dgyro_)) {
        TOBAS_FATAL("Failed to solve the mixing equation.");
        return;
      }

      // Fill the feedback message.
      setpoint->odom.accel.angular = tar_dgyro_;
    }

    *deflections_ = Eigen::VectorXd::Zero(drone_.fixed_wing->numControlSurfaces());
    for (size_t idx = 0; idx < drone_.fixed_wing->numControlSurfaces(); idx++) {
      deflections_->operator()(idx) = std::max(std::min(min_deflections_(idx), mixer_.getDeflection(idx)), max_deflections_(idx));
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
  tar_angle_.reset();

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
      deflections_->operator()(idx) = math::remap(manual_cmd->aileron, manual_cmd->MIN_DEFLECTION, manual_cmd->MAX_DEFLECTION, min_deflections_(idx), max_deflections_(idx));
    }
    else if (type == ControlSurfaceType::kElevator) {
      deflections_->operator()(idx) = math::remap(manual_cmd->elevator, manual_cmd->MIN_DEFLECTION, manual_cmd->MAX_DEFLECTION, min_deflections_(idx), max_deflections_(idx));
    }
    else if (type == ControlSurfaceType::kRudder) {
      deflections_->operator()(idx) = math::remap(manual_cmd->rudder, manual_cmd->MIN_DEFLECTION, manual_cmd->MAX_DEFLECTION, min_deflections_(idx), max_deflections_(idx));
    }
    else {
      deflections_->operator()(idx) = 0.0;
    }
  }
}

void ControllerNode::angleCmdCb(const tobas_command_msgs::AngleThrottle::ConstSharedPtr& cmd)
{
  if (!isCommandAccepted(cmd->priority)) {
    return;
  }

  // Check command range.
  if (std::abs(cmd->angle.roll) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (std::abs(cmd->angle.pitch) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }

  // TODO: Stop the outer control loop.

  // Create the command.
  if (!tar_angle_) {
    tar_angle_ = std::make_unique<kdl::Euler>();
  }

  // Update the command.
  *tar_angle_ = cmd->angle;
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
