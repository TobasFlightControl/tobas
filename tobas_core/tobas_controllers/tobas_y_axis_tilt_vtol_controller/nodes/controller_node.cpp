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

#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>
#include <tobas_command_msgs_adapter/accel_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>
#include <tobas_command_msgs/msg/roll_pitch_side_slip_throttle_vector.hpp>
#include <tobas_command_msgs/msg/vel_roll_side_slip_mode.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_y_axis_tilt_vtol_controller/mixer.hpp"
#include "tobas_y_axis_tilt_vtol_controller/transition_handler.hpp"
#include "tobas_y_axis_tilt_vtol_controller/translational_eom.hpp"

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
  // multicopter commands
  tobas_command_msgs::PosVelAccPitchYaw::UniquePtr multicopter_pos_cmd_ = nullptr;  // Position-control target value in the WCS.
  tobas_command_msgs::AccelPitchYaw::UniquePtr multicopter_acc_cmd_ = nullptr;      // Acceleration-control target value in the WCS.
  std::unique_ptr<kdl::Euler> tar_angle_ = nullptr;
  // fixed wing commands
  tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr fixed_wing_vel_cmd_ = nullptr;
  std::unique_ptr<RollPitchSideSlip> tar_roll_pitch_sideslip_ = nullptr;
  // shared commands
  std::unique_ptr<kdl::Vector> tar_gyro_ = nullptr;
  std::unique_ptr<kdl::Vector> tar_dgyro_ = nullptr;
  double thrust_ = 0.0; // Target thrust [N]
  double thrust_angle_ = 0.0; // Target thrust angle [rad]

  // Controller
  TranslationalEoM trans_eom_;
  TransitionHandler transition_handler_;
  Mixer mixer_;
  struct TranslationControlParameters
  {
    double hor_wn, ver_wn;      // [rad/s]
    double hor_zeta, ver_zeta;  // [-]
  } trans_ctrl_;
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
  ros2::SubscriberPtr<tobas_command_msgs::PosVelAccPitchYaw> multicopter_stabilize_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector> fixed_wing_manual_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::VelRollSideSlipMode> fixed_wing_stabilize_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool initialize();
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& priority);
  static kdl::Vector computeEulerError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy);

  // Parameter callbacks
  bool cruiseVelocityCb(const double& p);
  bool maxTiltVelocityCb(const double& p);
  bool horizontalNaturalFreqCb(const double& p);
  bool horizontalDampingRatioCb(const double& p);
  bool verticalNaturalFreqCb(const double& p);
  bool verticalDampingRatioCb(const double& p);
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
  void multicopterStabilizeCmdCb(const tobas_command_msgs::PosVelAccPitchYaw::ConstSharedPtr& cmd);
  void fixedWingManualCmdCb(const tobas_command_msgs::msg::RollPitchSideSlipThrottleVector::ConstSharedPtr& cmd);
  void fixedWingStabilizeCmdCb(const tobas_command_msgs::msg::VelRollSideSlipMode::ConstSharedPtr& cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(node::kController, nodeOptions_DParam(options)), mass_holder_(tree_), js_converter_(tree_), trans_eom_(drone_, tree_), transition_handler_(trans_eom_), mixer_(drone_, tree_)
{
  // setup beta_dot filter
  beta_dot_filter_.setCutoffFrequency(kBetaDotCutOffFreq);

  // Register dynamics parameters
  addDynamicDoubleParam("cruise_velocity", &self::cruiseVelocityCb, this, 5, 3, 1, 9, " m/s");
  addDynamicDoubleParam("max_tilt_velocity", &self::maxTiltVelocityCb, this, 15, 2, 1, 6, " deg/s");
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFreqCb, this, 0.2, 5, 1, 30, " rad/s");
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFreqCb, this, 0.2, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFreqCb, this, 1.0, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFreqCb, this, 0.2, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 0.1, 7, 1, 20);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 0.1, 10, 1, 20);
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
  multicopter_stabilize_cmd_sub_ = createSubscriber(topic::kPosVelAccPitchYawCmd, &self::multicopterStabilizeCmdCb, this);
  fixed_wing_manual_cmd_sub_ = createSubscriber(topic::kRollPitchSideSlipThrotVectorCmd, &self::fixedWingManualCmdCb, this);
  fixed_wing_stabilize_cmd_sub_ = createSubscriber(topic::kVelRollSideSlipModeCmd, &self::fixedWingStabilizeCmdCb, this);

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
  if (!trans_eom_.updateInternalDataStructures()) {
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

kdl::Vector ControllerNode::computeEulerError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy)
{
  // Note that a straight line connecting two Euler angles is not the shortest distance in rotation.
  const auto roll_err = algo::wrapPi(tar_rpy.roll - cur_rpy.roll);
  const auto pitch_err = algo::wrapPi(tar_rpy.pitch - cur_rpy.pitch);
  const auto yaw_err = algo::wrapPi(tar_rpy.yaw - cur_rpy.yaw);
  return { roll_err, pitch_err, yaw_err };
}

bool ControllerNode::cruiseVelocityCb(const double& p)
{
  transition_handler_.setCruiseVelocity(p);
  return true;
}

bool ControllerNode::maxTiltVelocityCb(const double& p)
{
  transition_handler_.setMaxTiltVelocity(st::deg2rad(p));
  return true;
}

bool ControllerNode::horizontalNaturalFreqCb(const double& p)
{
  trans_ctrl_.hor_wn = p;
  transition_handler_.setHorizontalNaturalFreq(p); // TODO: 別の関数に分ける
  return true;
}

bool ControllerNode::horizontalDampingRatioCb(const double& p)
{
  trans_ctrl_.hor_zeta = p;
  return true;
}

bool ControllerNode::verticalNaturalFreqCb(const double& p)
{
  trans_ctrl_.ver_wn = p;
  transition_handler_.setVerticalNatualFreq(p); // TODO: 別の関数に分ける
  return true;
}

bool ControllerNode::verticalDampingRatioCb(const double& p)
{
  trans_ctrl_.ver_zeta = p;
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
    multicopter_pos_cmd_.reset();
    multicopter_acc_cmd_.reset();
    fixed_wing_vel_cmd_.reset();
    tar_angle_.reset();
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
  const auto& cur_pos_W = odom_flu->odom.odom.frame.p;
  const auto& cur_rot = odom_flu->odom.odom.frame.M;
  const auto B_T_G =
    kdl::Frame(kdl::Rotation::Identity(), drone_.fixed_wing->vehicle.ref);  // base_link座標系で表したCoG座標系のFrame
  const auto cur_twist_G = B_T_G.inverse() * odom_flu->odom.odom.twist;
  const auto& cur_vel_G = cur_twist_G.vel;
  const auto& cur_gyro_G = cur_twist_G.rot;
  kdl::Vector cur_vel_G_frd;
  kdl::vectorFluToFrd(cur_vel_G, cur_vel_G_frd);
  const auto beta = angleOfSideSlip(cur_vel_G_frd.x(), cur_vel_G_frd.y(), cur_vel_G_frd.z());
  beta_dot_filter_.update(beta, dt);

  // Position controller.
  // multicopter like
  if (multicopter_pos_cmd_) {
    if (!multicopter_acc_cmd_) {
      multicopter_acc_cmd_ = std::make_unique<tobas_command_msgs::AccelPitchYaw>();
    }

    // Determine gains.
    const auto hor_wn = trans_ctrl_.hor_wn;
    const auto ver_wn = trans_ctrl_.ver_wn;
    const auto hor_kp = math::sqr(hor_wn);
    const auto ver_kp = math::sqr(ver_wn);
    const auto hor_kd = 2 * trans_ctrl_.hor_zeta * hor_wn;
    const auto ver_kd = 2 * trans_ctrl_.ver_zeta * ver_wn;
    const kdl::Vector kp(hor_kp, hor_kp, ver_kp);
    const kdl::Vector kd(hor_kd, hor_kd, ver_kd);

    // Compute error.
    const auto ep = multicopter_pos_cmd_->pos - cur_pos_W;
    const auto ed = multicopter_pos_cmd_->vel - cur_rot * cur_vel_G;

    // TODO: Accumulate integral error while airborne.

    // Compute target acceleration.
    multicopter_acc_cmd_->accel = multicopter_pos_cmd_->acc + kp.hadamard(ep) + kd.hadamard(ed);

    // Pass pitch and yaw through as-is.
    multicopter_acc_cmd_->pitch = multicopter_pos_cmd_->pitch;
    multicopter_acc_cmd_->yaw = multicopter_pos_cmd_->yaw;

    // Fill the feedback message.
    setpoint->odom.frame.p = multicopter_pos_cmd_->pos;
    setpoint->odom.twist.vel = cur_rot.inverse(multicopter_pos_cmd_->vel);

    // Acceleration controller.
    if (!tar_angle_) {
      tar_angle_ = std::make_unique<kdl::Euler>();
    }

    // Compute thrust and target attitude.
    if (!trans_eom_.solve(multicopter_acc_cmd_->accel, multicopter_acc_cmd_->pitch, multicopter_acc_cmd_->yaw, thrust_, thrust_angle_, *tar_angle_)) {
      TOBAS_FATAL("Failed to solve the translational EoM.");
      return;
    }

    // Fill the feedback message.
    setpoint->odom.accel.linear = cur_rot.inverse(multicopter_acc_cmd_->accel);
  }
  // fixed wing like
  if (fixed_wing_vel_cmd_) {
    if (!tar_roll_pitch_sideslip_) {
      tar_roll_pitch_sideslip_ = std::make_unique<RollPitchSideSlip>();
    }
    transition_handler_.solve(fixed_wing_vel_cmd_, cur_vel_G, cur_rot, tar_roll_pitch_sideslip_->roll, tar_roll_pitch_sideslip_->pitch, tar_roll_pitch_sideslip_->side_slip, thrust_, thrust_angle_);
  }

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
    {
      const auto rho = st::pressureToDensity(air_pressure_->pressure);
      if (!mixer_.solve(dt, rho, js_converter_.getPosition(), cur_gyro_G, cur_vel_G, *tar_dgyro_, thrust_, thrust_angle_)) {
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

void ControllerNode::multicopterStabilizeCmdCb(const tobas_command_msgs::PosVelAccPitchYaw::ConstSharedPtr& cmd)
{
  if (!isCommandAccepted(cmd->priority)) {
    return;
  }

  // Stop the fixed wing control.
  fixed_wing_vel_cmd_.reset();
  tar_roll_pitch_sideslip_.reset();

  // Create the command.
  if (!multicopter_pos_cmd_) {
    multicopter_pos_cmd_ = std::make_unique<tobas_command_msgs::PosVelAccPitchYaw>();
  }

  // Update transition handler
  transition_handler_.updateCommandType(TransitionHandler::CommandType::MULTICOPTER);

  // Update the command.
  *multicopter_pos_cmd_ = *cmd;
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

  // Stop the outer control loop.
  multicopter_pos_cmd_.reset();
  multicopter_acc_cmd_.reset();

  // Stop the fixed_wing like control
  fixed_wing_vel_cmd_.reset();
  tar_roll_pitch_sideslip_.reset();

  // Create the command.
  if (!tar_angle_) {
    tar_angle_ = std::make_unique<kdl::Euler>();
  }

  // Update transition handler
  transition_handler_.updateCommandType(TransitionHandler::CommandType::MANUAL_MULTICOPTER);

  // Update the command.
  *tar_angle_ = cmd->angle;
  const auto tar_thrust = mixer_.calcMaxThrust(cmd->thrust_angle) * std::clamp(cmd->throttle, kMinThrot, kMaxThrot);
  thrust_ = tar_thrust;
  thrust_angle_ = cmd->thrust_angle;
}

void ControllerNode::fixedWingStabilizeCmdCb(const tobas_command_msgs::msg::VelRollSideSlipMode::ConstSharedPtr& cmd)
{
  if (!isCommandAccepted(cmd->priority)) {
    return;
  }

  // Stop the multicopter control.
  multicopter_pos_cmd_.reset();
  multicopter_acc_cmd_.reset();
  tar_angle_.reset();

  // Create the command.
  if (!fixed_wing_vel_cmd_) {
    fixed_wing_vel_cmd_ = std::make_unique<tobas_command_msgs::msg::VelRollSideSlipMode>();
  }

  // Update transition handler
  switch (fixed_wing_vel_cmd_->mode)
  {
  case tobas_command_msgs::msg::VelRollSideSlipMode::FORWARD_FLIGHT:
    transition_handler_.updateCommandType(TransitionHandler::CommandType::FORWARD_FLIGHT);
    break;
  case tobas_command_msgs::msg::VelRollSideSlipMode::BACKWARD_TRANSITION:
    transition_handler_.updateCommandType(TransitionHandler::CommandType::BACKWARD_TRANSITION);
    break;
  default:
    TOBAS_ERROR("Undefined mode is commanded.");
    break;
  }

  // Update the command.
  *fixed_wing_vel_cmd_ = *cmd;
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

  // Stop the outer control loop.
  fixed_wing_vel_cmd_.reset();

  // Stop the multicopter like control
  multicopter_pos_cmd_.reset();
  multicopter_acc_cmd_.reset();
  tar_angle_.reset();

  // Create the command.
  if (!tar_roll_pitch_sideslip_) {
    tar_roll_pitch_sideslip_ = std::make_unique<RollPitchSideSlip>();
  }

  // Update transition handler
  transition_handler_.updateCommandType(TransitionHandler::CommandType::MANUAL_FIXED_WING);

  // Update the command.
  tar_roll_pitch_sideslip_->roll = cmd->roll;
  tar_roll_pitch_sideslip_->pitch = cmd->pitch;
  tar_roll_pitch_sideslip_->side_slip = cmd->side_slip;
  const auto tar_thrust = mixer_.calcMaxThrust(cmd->thrust_angle) * std::clamp(cmd->throttle, kMinThrot, kMaxThrot);
  thrust_ = tar_thrust;
  thrust_angle_ = cmd->thrust_angle;
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
