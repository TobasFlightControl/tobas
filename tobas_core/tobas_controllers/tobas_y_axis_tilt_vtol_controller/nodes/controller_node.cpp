// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_dsp/low_pass_filter.hpp>
#include <tobas_dsp/incomplete_differentiation.hpp>
#include <tobas_eigen_tools/core.hpp>
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
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>
#include <tobas_command_msgs/msg/roll_pitch_side_slip_throttle_vector.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_y_axis_tilt_vtol_controller/mixer.hpp"

namespace tobas
{
namespace y_axis_tilt_vtol
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
  TreeJointStateConverter js_converter_;

  dsp::IncompleteDifferentiation<double> beta_dot_filter_;

  // State
  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool js_received_ = false;
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
  std::unique_ptr<kdl::Rotation> tar_rot_ = nullptr;
  std::unique_ptr<RollPitchSideSlip> tar_roll_pitch_sideslip_ = nullptr;
  std::unique_ptr<kdl::Vector> tar_gyro_ = nullptr;
  std::unique_ptr<kdl::Vector> tar_dgyro_ = nullptr;
  double ux_ = 0.0, uz_ = 0.0;  // Target thrust in the body coordinate system.

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
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::AngleThrottleVector> multicopter_manual_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector> fixed_wing_manual_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool initialize();
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& priority);

  // Parameter callbacks
  bool attitudeNaturalFreqCb(const double& p);
  bool attitudeDampingRatioCb(const double& p);
  bool headingNaturalFreqCb(const double& p);
  bool headingDampingRatioCb(const double& p);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& pressure);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom_flu);
  void multicopterManualCmdCb(const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr& cmd);
  void fixedWingManualCmdCb(const tobas_command_msgs::msg::RollPitchSideSlipThrottleVector::ConstSharedPtr& cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(node::kController, nodeOptions_DParam(options)), mass_holder_(tree_), js_converter_(tree_), mixer_(drone_, tree_)
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
  multicopter_manual_cmd_sub_ = createSubscriber(topic::kAngleThrotVectorCmd, &self::multicopterManualCmdCb, this);
  fixed_wing_manual_cmd_sub_ = createSubscriber(topic::kRollPitchSideSlipThrotVectorCmd, &self::fixedWingManualCmdCb, this);

  // Register timers.
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::initialize()
{
  if (!mass_holder_.updateInternalDataStructures()) {
    return false;
  }
  if (!js_converter_.updateInternalDataStructures()) {
    return false;
  }
  if (!mixer_.updateInternalDataStructures()) {
    return false;
  }

  is_initialized_ = true;
  return true;
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

  if (drone->hasServoJoint()) {
    js_sub_ = createSubscriber(topic::kJointStates, &self::jointStateCb, this);
  }
  else {
    js_sub_.reset();
  }

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

void ControllerNode::jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js)
{
  // Assume that information for different joints may arrive in separate messages,
  // and convert to KDL inside the callback instead of storing the message itself.
  if (js_converter_.convert(*js) < 0) {
    TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());
    return;
  }

  js_received_ = true;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (!arming->data) {
    tar_rot_.reset();
    tar_roll_pitch_sideslip_.reset();
    tar_gyro_.reset();
    tar_dgyro_.reset();
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
  // multicopter like
  if (tar_rot_) {
    if (!tar_gyro_) {
      tar_gyro_ = std::make_unique<kdl::Vector>();
    }

    // Determine gains.
    const auto atti_wn = rot_ctrl_.atti_wn;
    const auto head_wn = rot_ctrl_.head_wn;
    const auto atti_angle_gain = atti_wn / rot_ctrl_.atti_zeta / 2;
    const auto head_angle_gain = head_wn / rot_ctrl_.head_zeta / 2;
    const kdl::Vector angle_gain(atti_angle_gain, atti_angle_gain, head_angle_gain);

    // Compute rotation error.
    const auto ep = (cur_rot.inverse() * *tar_rot_).getRot();

    // TODO: Accumulate integral error while airborne.

    // Compute target angular velocity.
    *tar_gyro_ = angle_gain.hadamard(ep);

    // Fill the feedback message.
    setpoint->odom.frame.M = *tar_rot_;

    // Angular velocity controller
    if (!tar_dgyro_) {
      tar_dgyro_ = std::make_unique<kdl::Vector>();
    }
    // Determine gains.
    const auto atti_rate_gain = atti_wn * rot_ctrl_.atti_zeta * 2;
    const auto head_rate_gain = head_wn * rot_ctrl_.head_zeta * 2;
    const kdl::Vector rate_gain(atti_rate_gain, atti_rate_gain, head_rate_gain);

    // Compute target angular acceleration.
    *tar_dgyro_ = rate_gain.hadamard(*tar_gyro_ - cur_gyro_G);

    // Fill the feedback message.
    setpoint->odom.twist.rot = *tar_gyro_;
  }
  // fixed wing like
  if (tar_roll_pitch_sideslip_) {
    if (!tar_gyro_) {
      tar_gyro_ = std::make_unique<kdl::Vector>();
    }

    // Determine gains.
    const auto atti_wn = rot_ctrl_.atti_wn;
    const auto head_wn = rot_ctrl_.head_wn;
    const auto atti_angle_gain = atti_wn / rot_ctrl_.atti_zeta / 2;
    const auto head_angle_gain = head_wn / rot_ctrl_.atti_zeta / 2;
    const kdl::Vector angle_gain(atti_angle_gain, atti_angle_gain, head_angle_gain);

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
    kdl::Vector ep(roll_err, pitch_err, beta_err);
    // std::cout << "beta : " << beta / M_PI * 180.0 << " [deg]" << std::endl;
    // std::cout << "beta_err : " << beta_err / M_PI * 180.0 << " [deg]" << std::endl;
    // std::cout << "beta_sp : " << tar_rpb.sideslip / M_PI * 180.0 << " [deg]" << std::endl;

    // TODO: Accumulate integral error while airborne.

    // Compute target Euler angle rates.
    *tar_gyro_ = angle_gain.hadamard(ep);

    // Fill the feedback message.
    setpoint->odom.frame.M =
      kdl::Euler(tar_roll_pitch_sideslip_->roll, tar_roll_pitch_sideslip_->pitch, 0.0).toRotation();

    // Angular velocity controller
    if (!tar_dgyro_) {
      tar_dgyro_ = std::make_unique<kdl::Vector>();
    }
    // Determine gains.
    const auto atti_rate_gain = atti_wn * rot_ctrl_.atti_zeta * 2;
    const auto head_rate_gain = head_wn * rot_ctrl_.head_zeta * 2;
    const kdl::Vector rate_gain(atti_rate_gain, atti_rate_gain, head_rate_gain);

    // Compute target angular acceleration.
    // roll = pitch = 0のときbeta = -yaw_frd = yaw_flu
    // TODO: 若干beta_dotが振動気味
    // std::cout << "tar_drpb_->z() : " << tar_drpb_->z() / M_PI * 180.0 << " [deg/s]" << std::endl;
    // std::cout << "beta_dot_filter_.getValue() : " << beta_dot_filter_.getValue() / M_PI * 180.0 << " [deg/s]" << std::endl;
    *tar_dgyro_ = kdl::Vector(
      atti_rate_gain * (tar_gyro_->x() - cur_gyro_G.x()),
      atti_rate_gain * (tar_gyro_->y() - cur_gyro_G.y()),
      head_rate_gain * (tar_gyro_->z() - beta_dot_filter_.getValue()));

    // Fill the feedback message.
    setpoint->odom.twist.rot = *tar_gyro_;
  }

  if (tar_dgyro_) {
    // Mixer.
    {
      const auto rho = st::pressureToDensity(air_pressure_->pressure);
      if (!mixer_.solve(dt, rho, airspeed, js_converter_.getPosition(), cur_gyro_G, *tar_dgyro_, ux_, uz_)) {
        TOBAS_FATAL("Failed to solve the mixing equation.");
        return;
      }

      // Fill the feedback message.
      setpoint->odom.accel.angular = *tar_dgyro_;
    }

    // Publish target thrust.
    auto tar_thrusts = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    tar_thrusts->header.stamp = cur_time;
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      tar_thrusts->thrusts.emplace_back();
      tar_thrusts->thrusts.back().link_name = rotor_it.first;
      tar_thrusts->thrusts.back().thrust = mixer_.getThrust(idx);
    }
    tar_thrusts_pub_->publish(std::move(tar_thrusts));

    // Publish joint command angles.
    auto tar_angles_msg = std::make_unique<tobas_msgs::msg::JointCommandArray>();
    tar_angles_msg->header.stamp = cur_time;
    // tilt joints
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      const auto& rotor = rotor_it.second;
      if (rotor->tilt_joint_name.empty()) {
        continue;
      }
      tar_angles_msg->commands.emplace_back();
      tar_angles_msg->commands.back().name = rotor->tilt_joint_name;
      tar_angles_msg->commands.back().data = mixer_.getTiltAngle(idx);
    }
    // control surfaces
    for (const auto& [idx, cs_item] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
      const auto& link_name = cs_item.first;
      const auto& joint = tree_.getSegment(link_name)->second.segment.joint();

      tar_angles_msg->commands.emplace_back();
      tar_angles_msg->commands.back().name = joint.name;
      tar_angles_msg->commands.back().data = std::clamp(mixer_.getDeflection(idx), joint.lower_limit, joint.upper_limit);
    }

    tar_angles_pub_->publish(std::move(tar_angles_msg));
  }

  // Publish the feedback message.
  setpoint_pub_->publish(std::move(setpoint));
}

void ControllerNode::multicopterManualCmdCb(const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr& cmd)
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

  // Stop the same level fixed_wing like control
  tar_roll_pitch_sideslip_.reset();

  // Create the command.
  if (!tar_rot_) {
    tar_rot_ = std::make_unique<kdl::Rotation>();
  }

  // Update the command.
  *tar_rot_ = cmd->angle.toRotation();
  const auto tar_thrust = mixer_.calcMaxThrust(cmd->thrust_angle) * std::clamp(cmd->throttle, kMinThrot, kMaxThrot);
  ux_ = tar_thrust * std::sin(cmd->thrust_angle);
  uz_ = tar_thrust * std::cos(cmd->thrust_angle);
}

void ControllerNode::fixedWingManualCmdCb(const tobas_command_msgs::msg::RollPitchSideSlipThrottleVector::ConstSharedPtr& cmd)
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

  // Stop the same level multicopter like control
  tar_rot_.reset();

  // Create the command.
  if (!tar_roll_pitch_sideslip_) {
    tar_roll_pitch_sideslip_ = std::make_unique<RollPitchSideSlip>();
  }

  // Update the command.
  tar_roll_pitch_sideslip_->roll = cmd->roll;
  tar_roll_pitch_sideslip_->pitch = cmd->pitch;
  tar_roll_pitch_sideslip_->side_slip = cmd->side_slip;
  const auto tar_thrust = mixer_.calcMaxThrust(cmd->thrust_angle) * std::clamp(cmd->throttle, kMinThrot, kMaxThrot);
  ux_ = tar_thrust * std::sin(cmd->thrust_angle);
  uz_ = tar_thrust * std::cos(cmd->thrust_angle);
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

  if (js_sub_ && !js_received_) {
    TOBAS_WARN("Waiting for \"", topic::kJointStates, "\".");
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
}  // namespace y_axis_tilt_vtol
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::y_axis_tilt_vtol::ControllerNode)
