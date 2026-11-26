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
#include <tobas_command_msgs/msg/vel_roll_side_slip_mode.hpp>
#include <tobas_command_msgs_adapter/accel_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>
#include <tobas_debug_msgs_adapter/multicopter_controller_feedback.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_y_axis_tilt_vtol_controller/constants.hpp"
#include "tobas_y_axis_tilt_vtol_controller/mixer_qp.hpp"
#include "tobas_y_axis_tilt_vtol_controller/transition_handler.hpp"
#include "tobas_y_axis_tilt_vtol_controller/translational_eom.hpp"

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
  bool is_velocity_accurate_ = false;
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
  tobas_command_msgs::PosVelAccPitchYaw::UniquePtr multicopter_pos_cmd_ =
    nullptr;  // Position-control target value in the WCS.
  tobas_command_msgs::AccelPitchYaw::UniquePtr multicopter_acc_cmd_ =
    nullptr;  // Acceleration-control target value in the WCS.
  std::unique_ptr<kdl::Euler> tar_angle_ = nullptr;
  // fixed wing commands
  tobas_command_msgs::msg::VelRollSideSlipMode::UniquePtr fixed_wing_vel_cmd_ = nullptr;
  std::unique_ptr<RollPitchSideSlip> tar_roll_pitch_sideslip_ = nullptr;
  // shared commands
  std::unique_ptr<kdl::Vector> tar_gyro_ = nullptr;
  std::unique_ptr<kdl::Vector> tar_dgyro_ = nullptr;
  std::unique_ptr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_ = nullptr;
  std::unique_ptr<tobas_msgs::msg::JointCommandArray> tar_joint_angles_ = nullptr;
  double thrust_ = 0.0;        // Target thrust [N]
  double thrust_angle_ = 0.0;  // Target thrust angle [rad]

  // Controller
  TranslationalEoM trans_eom_;
  TransitionHandler transition_handler_;
  QpMixer mixer_;
  double cruise_velocity_;       // [m/s]
  double throttle_gain_thresh_;  ///< [-]
  struct TranslationControlParameters
  {
    double hor_wn, ver_wn;      // [rad/s]
    double hor_zeta, ver_zeta;  // [-]
  } trans_ctrl_;
  struct RotationControlParameters
  {
    double multi_atti_wn, multi_head_wn;  // [rad/s]
    double fw_atti_wn, fw_sideslip_wn;    // [rad/s]
    double atti_zeta, head_zeta;          // [-]
    double atti_ki, head_ki;
    kdl::Vector ei = kdl::Vector::Zero();
  } rot_ctrl_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> tar_angles_pub_;
  ros2::PublisherPtr<tobas_msgs::OdometryStamped> setpoint_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::MulticopterControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressure> air_pressure_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::VehicleHealth> health_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::AngleThrottleVector> multicopter_angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::PosVelAccPitchYaw> multicopter_pos_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::AileElevRudThrottle> fixed_wing_manual_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::RollPitchSideSlipThrottleVector> fixed_wing_angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::VelRollSideSlipMode> fixed_wing_vel_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool initialize();
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& _priority);
  static kdl::Vector computeEulerError(const kdl::Euler& _cur_rpy, const kdl::Euler& _tar_rpy);

  // Parameter callbacks
  void cruiseVelocityCb(double _p);
  void minTransitionTimeCb(double _p);
  void horizontalNaturalFreqCb(double _p);
  void horizontalDampingRatioCb(double _p);
  void verticalNaturalFreqCb(double _p);
  void verticalDampingRatioCb(double _p);
  void multicopterAttitudeNaturalFreqCb(double _p);
  void fixedWingAttitudeNaturalFreqCb(double _p);
  void attitudeDampingRatioCb(double _p);
  void attitudeIGainCb(double _p);
  void headingNaturalFreqCb(double _p);
  void sideSlipNaturalFreqCb(double _p);
  void headingDampingRatioCb(double _p);
  void headingIGainCb(double _p);
  void throttleGainThresholdCb(double _p);

  void droneCb(const Drone::ConstSharedPtr& _drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& _tree);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& _js);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming);
  void airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& _pressure);
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& _health);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _odom_flu);
  void multicopterAngleCmdCb(const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr& _cmd);
  void multicopterPosCmdCb(const tobas_command_msgs::PosVelAccPitchYaw::ConstSharedPtr& _cmd);
  void fixedWingManualCmdCb(const tobas_command_msgs::msg::AileElevRudThrottle::ConstSharedPtr& _cmd);
  void fixedWingAngleCmdCb(const tobas_command_msgs::msg::RollPitchSideSlipThrottleVector::ConstSharedPtr& _cmd);
  void fixedWingVelCmdCb(const tobas_command_msgs::msg::VelRollSideSlipMode::ConstSharedPtr& _cmd);

  void checkTopicsTimerCb();
};

namespace
{
constexpr double kBetaDotCutOffFreq = 30.0;  // beta dot filterのカットオフ周波数
constexpr double kThrustClipValue = 1.0e-1;  // これより下の値はmixerに入れない
constexpr double kMinThrustAngle = 0.0;      // これより下の値はmixerに入れない
}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions& _options)
  : super(node::kController, nodeOptions_DParam(_options))
  , mass_holder_(tree_)
  , js_converter_(tree_)
  , trans_eom_(drone_, tree_)
  , transition_handler_(trans_eom_)
  , mixer_(drone_, tree_)
{
  // setup beta_dot filter
  beta_dot_filter_.setCutoffFrequency(kBetaDotCutOffFreq);

  // Register dynamics parameters
  addDynamicDoubleParam("cruise_velocity", &self::cruiseVelocityCb, this, 5, 3, 1, 9, " m/s");
  addDynamicDoubleParam("max_tilt_velocity", &self::minTransitionTimeCb, this, 1, 5, 1, 10, " s");
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFreqCb, this, 0.2, 5, 1, 30, " rad/s");
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFreqCb, this, 0.2, 10, 1, 30, " rad/s");
  addDynamicDoubleParam(
    "multicopter_attitude_natural_frequency", &self::multicopterAttitudeNaturalFreqCb, this, 1.0, 10, 1, 30, " rad/s");
  addDynamicDoubleParam(
    "fixed_wing_attitude_natural_frequency", &self::fixedWingAttitudeNaturalFreqCb, this, 1.0, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFreqCb, this, 0.2, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("sideslip_natural_frequency", &self::sideSlipNaturalFreqCb, this, 0.2, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 0.1, 7, 1, 20);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 0.1, 10, 1, 20);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 0.1, 10, 1, 20);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 0.1, 10, 1, 20);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0.1, 10, 0, 30);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0.01, 10, 0, 30);
  addDynamicDoubleParam("throttle_gain_threshold", &self::throttleGainThresholdCb, this, 1.0, 70, 0, 100, " %");

  // Register publishers.
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(topic::kRotorThrustsCmd);
  tar_angles_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd);
  setpoint_pub_ = createPublisher<tobas_msgs::OdometryStamped>(topic::kTrajSetpoint);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MulticopterControllerFeedback>(topic::kMRCtrlFeedback);

  // Register subscribers.
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  air_pressure_sub_ = createSubscriber(topic::kAirPressure, &self::airPressureCb, this);
  health_sub_ = createSubscriber(topic::kVehicleHealth, &self::healthCb, this);
  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
  multicopter_angle_cmd_sub_ = createSubscriber(topic::kAngleThrotVectorCmd, &self::multicopterAngleCmdCb, this);
  multicopter_pos_cmd_sub_ = createSubscriber(topic::kPosVelAccPitchYawCmd, &self::multicopterPosCmdCb, this);
  fixed_wing_manual_cmd_sub_ = createSubscriber(topic::kAileElevRudThrottleCmd, &self::fixedWingManualCmdCb, this);
  fixed_wing_angle_cmd_sub_ =
    createSubscriber(topic::kRollPitchSideSlipThrotVectorCmd, &self::fixedWingAngleCmdCb, this);
  fixed_wing_vel_cmd_sub_ = createSubscriber(topic::kVelRollSideSlipModeCmd, &self::fixedWingVelCmdCb, this);

  // Register timers.
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::initialize()
{
  mass_holder_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
  trans_eom_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

  transition_handler_.setAoARange(drone_.fixed_wing->vehicle.alpha_limit);

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

void ControllerNode::cruiseVelocityCb(double _p)
{
  cruise_velocity_ = _p;
  transition_handler_.setCruiseVelocity(_p);
}

void ControllerNode::minTransitionTimeCb(double _p)
{
  transition_handler_.setMinTransitionTime(_p);
}

void ControllerNode::horizontalNaturalFreqCb(double _p)
{
  trans_ctrl_.hor_wn = _p;
  transition_handler_.setHorizontalNaturalFreq(_p);  // TODO: 別の関数に分ける
}

void ControllerNode::horizontalDampingRatioCb(double _p)
{
  trans_ctrl_.hor_zeta = _p;
}

void ControllerNode::verticalNaturalFreqCb(double _p)
{
  trans_ctrl_.ver_wn = _p;
  transition_handler_.setVerticalNatualFreq(_p);  // TODO: 別の関数に分ける
}

void ControllerNode::verticalDampingRatioCb(double _p)
{
  trans_ctrl_.ver_zeta = _p;
}

void ControllerNode::multicopterAttitudeNaturalFreqCb(double _p)
{
  rot_ctrl_.multi_atti_wn = _p;
}

void ControllerNode::fixedWingAttitudeNaturalFreqCb(double _p)
{
  rot_ctrl_.fw_atti_wn = _p;
}

void ControllerNode::attitudeDampingRatioCb(double _p)
{
  rot_ctrl_.atti_zeta = _p;
}

void ControllerNode::attitudeIGainCb(double _p)
{
  rot_ctrl_.atti_ki = _p;
}

void ControllerNode::headingNaturalFreqCb(double _p)
{
  rot_ctrl_.multi_head_wn = _p;
}

void ControllerNode::sideSlipNaturalFreqCb(double _p)
{
  rot_ctrl_.fw_sideslip_wn = _p;
}

void ControllerNode::headingDampingRatioCb(double _p)
{
  rot_ctrl_.head_zeta = _p;
}

void ControllerNode::headingIGainCb(double _p)
{
  rot_ctrl_.head_ki = _p;
}

void ControllerNode::throttleGainThresholdCb(double _p)
{
  throttle_gain_thresh_ = _p / 100.0;
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
    rot_ctrl_.ei.setZero();

    multicopter_pos_cmd_.reset();
    multicopter_acc_cmd_.reset();
    fixed_wing_vel_cmd_.reset();
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

void ControllerNode::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& _health)
{
  const auto passed = tobas_msgs::msg::VehicleHealth::PASSED;
  const auto is_velocity_accurate =
    (_health->horizontal_position_accuracy == passed) && (_health->vertical_position_accuracy == passed);
  mixer_.setVelocityReliability(is_velocity_accurate);
  is_velocity_accurate_ = is_velocity_accurate;
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

  // Create the feedback message.
  auto feedback = std::make_unique<tobas_debug_msgs::MulticopterControllerFeedback>();
  feedback->header.stamp = cur_time;

  // Aliases.
  const auto& cur_pos_W = _odom_flu->odom.odom.frame.p;
  const auto& cur_rot = _odom_flu->odom.odom.frame.M;
  const auto B_T_G =
    kdl::Frame(kdl::Rotation::Identity(), drone_.fixed_wing->vehicle.ref);  // base_link座標系で表したCoG座標系のFrame
  const auto cur_twist_G = B_T_G.inverse() * _odom_flu->odom.odom.twist;
  const auto& cur_vel_G = cur_twist_G.vel;
  const auto& cur_gyro_G = cur_twist_G.rot;
  kdl::Vector cur_vel_G_frd;
  kdl::vectorFluToFrd(cur_vel_G, cur_vel_G_frd);
  const auto airspeed = is_velocity_accurate_ ? cur_vel_G.norm() : 0.0;

  // Updates.
  const auto beta = cur_vel_G_frd.x() > kAngleCalcThreshold ?
                      angleOfSideSlip(cur_vel_G_frd.x(), cur_vel_G_frd.y(), cur_vel_G_frd.z()) :
                      0.0;
  beta_dot_filter_.update(beta, dt);

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
    if (!trans_eom_.solve(
          multicopter_acc_cmd_->accel,
          multicopter_acc_cmd_->pitch,
          multicopter_acc_cmd_->yaw,
          thrust_,
          thrust_angle_,
          *tar_angle_)) {
      TOBAS_FATAL("Failed to solve the translational EoM.");
      return;
    }

    if (thrust_angle_ < kMinThrustAngle) {
      thrust_angle_ = kMinThrustAngle;
      if (!trans_eom_.solveMulticopterDynamicsWithThrustAngleFixed(
            multicopter_acc_cmd_->accel, multicopter_acc_cmd_->yaw, kMinThrustAngle, thrust_, *tar_angle_)) {
        TOBAS_FATAL("Failed to solve the translational EoM.");
        return;
      }
    }

    // Fill the feedback message.
    setpoint->odom.accel.linear = cur_rot.inverse(multicopter_acc_cmd_->accel);
  }
  // fixed wing like
  if (fixed_wing_vel_cmd_) {
    if (!tar_roll_pitch_sideslip_) {
      tar_roll_pitch_sideslip_ = std::make_unique<RollPitchSideSlip>();
    }
    transition_handler_.solve(
      fixed_wing_vel_cmd_,
      cur_vel_G,
      cur_rot,
      tar_roll_pitch_sideslip_->roll,
      tar_roll_pitch_sideslip_->pitch,
      tar_roll_pitch_sideslip_->side_slip,
      thrust_,
      thrust_angle_);
  }

  // Attitude controller.
  // multicopter like
  if (tar_angle_) {
    if (!tar_gyro_) {
      tar_gyro_ = std::make_unique<kdl::Vector>();
    }

    // When target thrust is below the threshold defined as a fraction of weight,
    // reduce the controller natural frequency as thrust decreases.
    const auto thrust_thresh = mass_holder_.getMass() * st::kGravity * throttle_gain_thresh_;
    const auto land_suspect = (thrust_ < thrust_thresh);
    const auto gain_throt = land_suspect ? thrust_ / thrust_thresh : 1.0;

    // Determine gains.
    const auto r = std::min(airspeed / cruise_velocity_, 1.0);
    const auto atti_wn = ((1.0 - r) * rot_ctrl_.multi_atti_wn + r * rot_ctrl_.fw_atti_wn) * gain_throt;
    const auto head_wn = ((1.0 - r) * rot_ctrl_.multi_head_wn + r * rot_ctrl_.fw_sideslip_wn) * gain_throt;
    const auto atti_angle_gain = atti_wn / rot_ctrl_.atti_zeta / 2;
    const auto head_angle_gain = head_wn / rot_ctrl_.head_zeta / 2;
    const auto atti_ki = rot_ctrl_.atti_ki * gain_throt;
    const auto head_ki = rot_ctrl_.head_ki * gain_throt;
    const kdl::Vector angle_gain(atti_angle_gain, atti_angle_gain, head_angle_gain);
    const kdl::Vector ki(atti_ki, atti_ki, head_ki);

    // Compute rotation error. yaw角の大きなズレを考慮外にするためEuler角による引き算を行う
    const kdl::Euler cur_rpy(cur_rot);
    const auto ep = computeEulerError(cur_rpy, *tar_angle_);

    // Accumulate integral error while airborne.
    if (!land_suspect) {
      for (int i = 0; i < 3; ++i) {
        if (ki(i) > 0.0) {
          rot_ctrl_.ei(i) += ep(i) * dt;
        }
        else {
          rot_ctrl_.ei(i) = 0.0;
        }
      }
    }

    // Compute target Euler angle rates.
    const auto tar_drpy = angle_gain.hadamard(ep) + ki.hadamard(rot_ctrl_.ei);

    // Convert Euler angle rates to gyro values.
    *tar_gyro_ = eigen::angvelFromEulerrateLocal(tar_drpy.data, cur_rpy.roll, cur_rpy.pitch);

    // Fill the feedback message.
    setpoint->odom.frame.M = tar_angle_->toRotation();
    feedback->angle_integral_error = rot_ctrl_.ei;

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
    // 固定翼時は推力によらず浮いている
    const auto r = std::min(airspeed / cruise_velocity_, 1.0);
    const auto atti_wn = ((1.0 - r) * rot_ctrl_.multi_atti_wn + r * rot_ctrl_.fw_atti_wn);
    const auto head_wn = ((1.0 - r) * rot_ctrl_.multi_head_wn + r * rot_ctrl_.fw_sideslip_wn);
    const auto atti_angle_gain = atti_wn / rot_ctrl_.atti_zeta / 2;
    const auto head_angle_gain = head_wn / rot_ctrl_.atti_zeta / 2;
    const auto atti_ki = rot_ctrl_.atti_ki;
    const auto head_ki = rot_ctrl_.head_ki;
    const kdl::Vector angle_gain(atti_angle_gain, atti_angle_gain, head_angle_gain);
    const kdl::Vector ki(atti_ki, atti_ki, head_ki);

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

    // Accumulate integral error while airborne.
    for (int i = 0; i < 3; ++i) {
      if (ki(i) > 0.0) {
        rot_ctrl_.ei(i) += ep(i) * dt;
      }
      else {
        rot_ctrl_.ei(i) = 0.0;
      }
    }

    // Compute target Euler angle rates.
    const auto tar_drpy = angle_gain.hadamard(ep) + ki.hadamard(rot_ctrl_.ei);

    // Convert Euler angle rates to gyro values.
    // マルチコプタの際とは違いRが無制御なので計算方法が異なる
    // \dot{\phi} = P + Q \sin\Phi\tan\Theta + R \cos\Phi\tan\Theta
    // \dot{\theta} = Q \cos\Phi - R\sin\Phi
    // をRを現在の角速度としてP, Qに関して逆解きする
    Eigen::Matrix2d C;
    C << 1, -std::tan(cur_rpy.roll) * std::tan(cur_rpy.pitch), 0, 1.0 / std::cos(cur_rpy.roll);
    const Eigen::Vector2d tar_gyro_xy =
      C * Eigen::Vector2d(
            tar_drpy.x() - cur_gyro_G.z() * std::cos(cur_rpy.roll) * std::tan(cur_rpy.pitch),
            tar_drpy.y() + cur_gyro_G.z() * std::sin(cur_rpy.roll));
    tar_gyro_->x() = tar_gyro_xy.x();
    tar_gyro_->y() = tar_gyro_xy.y();
    tar_gyro_->z() = tar_drpy.z();

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
  feedback_pub_->publish(std::move(feedback));
}

void ControllerNode::multicopterPosCmdCb(const tobas_command_msgs::PosVelAccPitchYaw::ConstSharedPtr& _cmd)
{
  if (!isCommandAccepted(_cmd->priority)) {
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
  *multicopter_pos_cmd_ = *_cmd;
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
  *tar_angle_ = _cmd->angle;
  const auto tar_thrust = mixer_.calcMaxThrust(_cmd->thrust_angle) * std::clamp(_cmd->throttle, kMinThrot, kMaxThrot);
  thrust_ = tar_thrust;
  thrust_angle_ = _cmd->thrust_angle;
}

void ControllerNode::fixedWingVelCmdCb(const tobas_command_msgs::msg::VelRollSideSlipMode::ConstSharedPtr& _cmd)
{
  if (!isCommandAccepted(_cmd->priority)) {
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
  switch (fixed_wing_vel_cmd_->mode) {
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
  *fixed_wing_vel_cmd_ = *_cmd;
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
  multicopter_pos_cmd_.reset();
  multicopter_acc_cmd_.reset();
  tar_angle_.reset();
  fixed_wing_vel_cmd_.reset();
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
