// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/ros.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_dsp/incomplete_differentiation.hpp>
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
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_command_msgs/msg/aile_elev_rud_throttle.hpp>
#include <tobas_command_msgs/msg/roll_pitch_side_slip_throttle_vector.hpp>
#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_y_axis_tilt_vtol_controller/constants.hpp"
#include "tobas_y_axis_tilt_vtol_controller/mixer_qp.hpp"

namespace tobas::y_axis_tilt_vtol
{
class ControllerNode : public BaseNode
{
  using self = ControllerNode;
  using super = BaseNode;

public:
  explicit ControllerNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

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
  // multicopter commands
  std::unique_ptr<kdl::Euler> tar_angle_ = nullptr;
  // fixed wing commands
  std::unique_ptr<RollPitchSideSlip> tar_roll_pitch_sideslip_ = nullptr;
  // shared commands
  std::unique_ptr<kdl::Vector> tar_gyro_ = nullptr;
  std::unique_ptr<kdl::Vector> tar_dgyro_ = nullptr;
  std::unique_ptr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_ = nullptr;
  std::unique_ptr<tobas_msgs::msg::JointCommandArray> tar_joint_angles_ = nullptr;
  double thrust_ = 0.0;        // Target thrust [N]
  double thrust_angle_ = 0.0;  // Target thrust angle [rad]

  // Controller
  QpMixer mixer_;
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
  ros2::SubscriberPtr<tobas_command_msgs::AngleThrottleVector> multicopter_angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::AileElevRudThrottle> fixed_wing_manual_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector> fixed_wing_angle_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool initialize();
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& _priority);
  static kdl::Vector computeEulerError(const kdl::Euler& _cur_rpy, const kdl::Euler& _tar_rpy);

  // Parameter callbacks
  void attitudeNaturalFreqCb(double _p);
  void attitudeDampingRatioCb(double _p);
  void headingNaturalFreqCb(double _p);
  void headingDampingRatioCb(double _p);

  void droneCb(const Drone::ConstSharedPtr& _drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& _tree);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& _js);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming);
  void airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& _pressure);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _odom_flu);
  void multicopterAngleCmdCb(const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr& _cmd);
  void fixedWingManualCmdCb(const tobas_command_msgs::msg::AileElevRudThrottle::ConstSharedPtr& _cmd);
  void fixedWingAngleCmdCb(const tobas_command_msgs::msg::RollPitchSideSlipThrottleVector::ConstSharedPtr& _cmd);

  void checkTopicsTimerCb();
};

namespace
{
constexpr double kBetaDotCutOffFreq = 30.0;  // beta dot filterのカットオフ周波数
constexpr double kThrustClipValue = 1.0e-1;  // これより下の値はmixerに入れない
}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions& _options)
  : super(node::kController, nodeOptions_DParam(_options))
  , mass_holder_(tree_)
  , js_converter_(tree_)
  , mixer_(drone_, tree_)
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
  multicopter_angle_cmd_sub_ = createSubscriber(topic::kAngleThrotVectorCmd, &self::multicopterAngleCmdCb, this);
  fixed_wing_manual_cmd_sub_ = createSubscriber(topic::kAileElevRudThrottleCmd, &self::fixedWingManualCmdCb, this);
  fixed_wing_angle_cmd_sub_ =
    createSubscriber(topic::kRollPitchSideSlipThrotVectorCmd, &self::fixedWingAngleCmdCb, this);

  // Register timers.
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::initialize()
{
  mass_holder_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

  is_initialized_ = true;
  return true;
}

bool ControllerNode::isCommandAccepted(const tobas_command_msgs::msg::Priority& _priority)
{
  if (!topics_received_) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because some topics have not been received yet.");
    return false;
  }

  if (!arming_->data) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because the vehicle is disarmed.");
    return false;
  }

  if (!cmd_priority_handler_.update(_priority.data, now())) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because of the its _priority.");
    return false;
  }

  return true;
}

kdl::Vector ControllerNode::computeEulerError(const kdl::Euler& _cur_rpy, const kdl::Euler& _tar_rpy)
{
  // Note that a straight line connecting two Euler angles is not the shortest distance in rotation.
  const auto roll_err = algo::wrapPi(_tar_rpy.roll - _cur_rpy.roll);
  const auto pitch_err = algo::wrapPi(_tar_rpy.pitch - _cur_rpy.pitch);
  const auto yaw_err = algo::wrapPi(_tar_rpy.yaw - _cur_rpy.yaw);
  return { roll_err, pitch_err, yaw_err };
}

void ControllerNode::attitudeNaturalFreqCb(double _p)
{
  rot_ctrl_.atti_wn = _p;
}

void ControllerNode::attitudeDampingRatioCb(double _p)
{
  rot_ctrl_.atti_zeta = _p;
}

void ControllerNode::headingNaturalFreqCb(double _p)
{
  rot_ctrl_.head_wn = _p;
}

void ControllerNode::headingDampingRatioCb(double _p)
{
  rot_ctrl_.head_zeta = _p;
}

void ControllerNode::droneCb(const Drone::ConstSharedPtr& _drone)
{
  drone_ = *_drone;

  if (_drone->hasServoJoint()) {
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

void ControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& _tree)
{
  tree_ = *_tree;

  if (drone_received_) {
    if (!initialize()) {
      TOBAS_FATAL("Error occurred while initializing controller.");
      return;
    }
  }

  tree_received_ = true;
}

void ControllerNode::jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& _js)
{
  // Assume that information for different joints may arrive in separate messages,
  // and convert to KDL inside the callback instead of storing the message itself.
  if (const auto result = js_converter_.convert(*_js); !result) {
    TOBAS_ERROR("Joint state converter failed: ", result.error());
    return;
  }

  js_received_ = true;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming)
{
  arming_ = _arming;

  if (!_arming->data) {
    tar_angle_.reset();
    tar_roll_pitch_sideslip_.reset();
    tar_gyro_.reset();
    tar_dgyro_.reset();
  }
}

void ControllerNode::airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& _pressure)
{
  air_pressure_ = _pressure;
}

void ControllerNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _odom_flu)
{
  if (!odom_flu_) {
    odom_flu_ = _odom_flu;
    return;
  }

  // Compute elapsed time and update odometry.
  const auto& cur_time = _odom_flu->header.stamp;
  const auto dt = (_odom_flu->header.stamp - odom_flu_->header.stamp).seconds();
  odom_flu_ = _odom_flu;

  // Create the setpoint message.
  auto setpoint = std::make_unique<tobas_msgs::OdometryStamped>();
  setpoint->header.stamp = cur_time;
  setpoint->odom.setNaN();

  // Aliases.
  const auto& cur_rot = _odom_flu->odom.odom.frame.M;
  const auto B_T_G =
    kdl::Frame(kdl::Rotation::Identity(), drone_.fixed_wing->vehicle.ref);  // base_link座標系で表したCoG座標系のFrame
  const auto cur_twist_G = B_T_G.inverse() * _odom_flu->odom.odom.twist;
  const auto& cur_vel_G = cur_twist_G.vel;
  const auto& cur_gyro_G = cur_twist_G.rot;
  kdl::Vector cur_vel_G_frd;
  kdl::vectorFluToFrd(cur_vel_G, cur_vel_G_frd);

  // Updates.
  const auto beta = cur_vel_G_frd.x() > kAngleCalcThreshold ?
                      angleOfSideSlip(cur_vel_G_frd.x(), cur_vel_G_frd.y(), cur_vel_G_frd.z()) :
                      0.0;
  beta_dot_filter_.update(beta, dt);

  // Attitude controller.
  // multicopter like
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

    // Compute rotation error. yaw角の大きなズレを考慮外にするためEuler角による引き算を行う
    const kdl::Euler cur_rpy(cur_rot);
    const auto ep = computeEulerError(cur_rpy, *tar_angle_);

    // TODO: Accumulate integral error while airborne.

    // Compute target Euler angle rates.
    const auto tar_drpy = angle_gain.hadamard(ep);

    // Convert Euler angle rates to gyro values.
    *tar_gyro_ = eigen::angvelFromEulerrateLocal(tar_drpy.data, cur_rpy.roll, cur_rpy.pitch);

    // Fill the feedback message.
    setpoint->odom.frame.M = tar_angle_->toRotation();

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
    // TODO: betaに関して制御能力が足りず偏差が残りがち
    const kdl::Euler cur_rpy(cur_rot);
    const auto roll_err = algo::wrapPi(tar_rpb.roll - cur_rpy.roll);
    const auto pitch_err = algo::wrapPi(tar_rpb.pitch - cur_rpy.pitch);
    const auto beta_err = algo::wrapPi(tar_rpb.side_slip - beta);
    kdl::Vector ep(roll_err, pitch_err, beta_err);

    // TODO: Accumulate integral error while airborne.

    // Compute target Euler angle rates.
    const auto tar_drpy = angle_gain.hadamard(ep);

    // Convert Euler angle rates to gyro values.
    *tar_gyro_ = eigen::angvelFromEulerrateLocal(tar_drpy.data, cur_rpy.roll, cur_rpy.pitch);

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
    *tar_dgyro_ = kdl::Vector(
      atti_rate_gain * (tar_gyro_->x() - cur_gyro_G.x()),
      atti_rate_gain * (tar_gyro_->y() - cur_gyro_G.y()),
      head_rate_gain * (tar_gyro_->z() - beta_dot_filter_.getValue()));

    // Fill the feedback message.
    setpoint->odom.twist.rot = *tar_gyro_;
  }

  if (tar_dgyro_) {
    // Mixer.
    const auto rho = st::pressureToDensity(air_pressure_->pressure);
    const auto mixer_result = mixer_.solve(
      dt,
      rho,
      js_converter_.getPosition(),
      cur_gyro_G,
      cur_vel_G,
      *tar_dgyro_,
      std::max(thrust_, kThrustClipValue),
      thrust_angle_);
    if (!mixer_result) {
      TOBAS_FATAL("Failed to solve the mixing equation.");
      return;
    }

    // Fill the feedback message.
    setpoint->odom.accel.angular = *tar_dgyro_;

    // Setup target thrust.
    if (!tar_thrusts_) {
      tar_thrusts_ = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    }

    tar_thrusts_->header.stamp = cur_time;
    tar_thrusts_->thrusts.clear();
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      tar_thrusts_->thrusts.emplace_back();
      tar_thrusts_->thrusts.back().link_name = rotor_it.first;
      tar_thrusts_->thrusts.back().thrust = mixer_result->thrusts(idx);
    }

    // Setup joint command angles.
    if (!tar_joint_angles_) {
      tar_joint_angles_ = std::make_unique<tobas_msgs::msg::JointCommandArray>();
    }

    tar_joint_angles_->header.stamp = cur_time;
    tar_joint_angles_->commands.clear();
    // tilt joints
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      const auto& rotor = rotor_it.second;
      if (rotor->tilt_joint_name.empty()) {
        continue;
      }
      tar_joint_angles_->commands.emplace_back();
      tar_joint_angles_->commands.back().name = rotor->tilt_joint_name;
      tar_joint_angles_->commands.back().data = mixer_result->tilt_angles(idx);
    }
    // control surfaces
    for (const auto& [idx, cs_item] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
      const auto& link_name = cs_item.first;
      const auto& joint = tree_.getSegment(link_name)->second.segment.joint();
      tar_joint_angles_->commands.emplace_back();
      tar_joint_angles_->commands.back().name = joint.name;
      tar_joint_angles_->commands.back().data =
        std::clamp(mixer_result->cs_deflections(idx), joint.lower_limit, joint.upper_limit);
    }
  }

  // Publish commands (including manual control commands)
  if (tar_thrusts_) {
    tar_thrusts_pub_->publish(std::move(tar_thrusts_));
  }
  if (tar_joint_angles_) {
    tar_angles_pub_->publish(std::move(tar_joint_angles_));
  }

  // Publish the feedback message.
  setpoint_pub_->publish(std::move(setpoint));
}

void ControllerNode::multicopterAngleCmdCb(const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr& _cmd)
{
  if (!isCommandAccepted(_cmd->priority)) {
    return;
  }

  // Check command range.
  if (std::abs(_cmd->angle.roll) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (std::abs(_cmd->angle.pitch) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }

  tar_roll_pitch_sideslip_.reset();

  // Create the command.
  if (!tar_angle_) {
    tar_angle_ = std::make_unique<kdl::Euler>();
  }

  // Update the command.
  *tar_angle_ = _cmd->angle;
  const auto tar_thrust = mixer_.calcMaxThrust(_cmd->thrust_angle) * std::clamp(_cmd->throttle, kMinThrot, kMaxThrot);
  thrust_ = tar_thrust;
  thrust_angle_ = _cmd->thrust_angle;
}

void ControllerNode::fixedWingAngleCmdCb(
  const tobas_command_msgs::msg::RollPitchSideSlipThrottleVector::ConstSharedPtr& _cmd)
{
  if (!isCommandAccepted(_cmd->priority)) {
    return;
  }

  // Check command range.
  if (std::abs(_cmd->roll) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (std::abs(_cmd->pitch) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }

  tar_angle_.reset();

  // Create the command.
  if (!tar_roll_pitch_sideslip_) {
    tar_roll_pitch_sideslip_ = std::make_unique<RollPitchSideSlip>();
  }

  // Update the command.
  tar_roll_pitch_sideslip_->roll = _cmd->roll;
  tar_roll_pitch_sideslip_->pitch = _cmd->pitch;
  tar_roll_pitch_sideslip_->side_slip = _cmd->side_slip;
  const auto tar_thrust = mixer_.calcMaxThrust(_cmd->thrust_angle) * std::clamp(_cmd->throttle, kMinThrot, kMaxThrot);
  thrust_ = tar_thrust;
  thrust_angle_ = _cmd->thrust_angle;
}

void ControllerNode::fixedWingManualCmdCb(const tobas_command_msgs::msg::AileElevRudThrottle::ConstSharedPtr& _cmd)
{
  if (!isCommandAccepted(_cmd->priority)) {
    return;
  }

  // Stop the outer control loop.
  tar_angle_.reset();
  tar_roll_pitch_sideslip_.reset();
  tar_dgyro_.reset();
  mixer_.registerInternalDataReset();

  // Create the command.
  if (!tar_thrusts_) {
    tar_thrusts_ = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
  }
  if (!tar_joint_angles_) {
    tar_joint_angles_ = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  }

  // Setup thrusts
  const auto cur_time = _cmd->header.stamp;
  if (!tar_thrusts_) {
    tar_thrusts_ = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
  }

  tar_thrusts_->header.stamp = cur_time;
  tar_thrusts_->thrusts.clear();
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    tar_thrusts_->thrusts.emplace_back();
    const auto link_name = rotor_it.first;
    const auto rotor = rotor_it.second;
    tar_thrusts_->thrusts.back().link_name = link_name;
    tar_thrusts_->thrusts.back().thrust =
      rotor->tilt_joint_name.empty() ? 0.0 : drone_.prop->maxThrust(link_name) * _cmd->throttle;
  }

  // Setup joint command angles.
  if (!tar_joint_angles_) {
    tar_joint_angles_ = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  }
  tar_joint_angles_->header.stamp = cur_time;
  tar_joint_angles_->commands.clear();
  // tilt joints
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    if (rotor->tilt_joint_name.empty()) {
      continue;
    }
    tar_joint_angles_->commands.emplace_back();
    tar_joint_angles_->commands.back().name = rotor->tilt_joint_name;
    const auto forward_tilt_angle = mixer_.getForwardTiltAngle(idx);
    if (!forward_tilt_angle) {
      TOBAS_ERROR(forward_tilt_angle.error());
    }
    tar_joint_angles_->commands.back().data = forward_tilt_angle.value();
  }
  // control surfaces
  for (const auto& [idx, cs_item] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& link_name = cs_item.first;
    const auto& joint = tree_.getSegment(link_name)->second.segment.joint();
    const auto type = cs_item.second.type;
    double command = 0.0;
    if (type == ControlSurfaceType::kAileron) {
      command = _cmd->aileron;
    }
    else if (type == ControlSurfaceType::kElevator) {
      command = _cmd->elevator;
    }
    else if (type == ControlSurfaceType::kRudder) {
      command = _cmd->rudder;
    }
    tar_joint_angles_->commands.emplace_back();
    tar_joint_angles_->commands.back().name = joint.name;
    tar_joint_angles_->commands.back().data =
      math::remap(command, _cmd->MIN_DEFLECTION, _cmd->MAX_DEFLECTION, joint.lower_limit, joint.upper_limit);
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
}  // namespace tobas::y_axis_tilt_vtol

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::y_axis_tilt_vtol::ControllerNode)
