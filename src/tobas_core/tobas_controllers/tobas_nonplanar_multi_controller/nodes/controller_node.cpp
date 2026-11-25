// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <optional>
#include <ranges>

#include <tobas_constants/ros.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/command_priority_handler.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_command_msgs_adapter/accel.hpp>
#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc.hpp>
#include <tobas_command_msgs_adapter/rate.hpp>
#include <tobas_debug_msgs_adapter/multicopter_controller_feedback.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry_stamped.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_nonplanar_multi_controller/mixer_qp.hpp"

namespace tobas::nonplanar_multicopter
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
  bool drone_received_ = false;
  bool tree_received_ = false;

  TreeJointStateConverter js_converter_;

  // Static parameters
  bool do_dist_comp_trans_;
  bool do_dist_comp_rot_;

  // Controllers
  QpMixer mixer_;
  double throttle_gain_thresh_;  ///< [-]
  struct TranslationControlParameters
  {
    double hor_wn, ver_wn;      ///< [rad/s]
    double hor_zeta, ver_zeta;  ///< [-]
    double hor_ki, ver_ki;
    double hor_max_i_acc, ver_max_i_acc;
    kdl::Vector ei = kdl::Vector::Zero();
  } trans_ctrl_;
  struct RotationControlParameters
  {
    double atti_wn, head_wn;      ///< [rad/s]
    double atti_zeta, head_zeta;  ///< [-]
    double atti_ki, head_ki;
    kdl::Vector ei = kdl::Vector::Zero();
  } rot_ctrl_;

  // Mutable variables
  bool js_received_ = false;
  bool topics_received_ = false;
  CommandPriorityHandler cmd_priority_handler_;
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  // Command
  std::optional<tobas_command_msgs::PosVelAcc> pos_cmd_;
  std::optional<tobas_command_msgs::Accel> acc_cmd_;
  std::optional<tobas_command_msgs::Angle> angle_cmd_;
  std::optional<tobas_command_msgs::Rate> rate_cmd_;
  std::optional<kdl::Vector> tar_dgyro_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_msgs::OdometryStamped> setpoint_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::MulticopterControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::PosVelAcc> pos_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::Accel> acc_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::Angle> angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::Rate> rate_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  void updateInternalDataStructures();
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& _priority);

  void horizontalNaturalFreqCb(double _p);
  void horizontalDampingRatioCb(double _p);
  void horizontalIGainCb(double _p);
  void horizontalIMaxAccelCb(double _p);
  void verticalNaturalFreqCb(double _p);
  void verticalDampingRatioCb(double _p);
  void verticalIGainCb(double _p);
  void verticalIMaxAccelCb(double _p);
  void attitudeNaturalFreqCb(double _p);
  void attitudeDampingRatioCb(double _p);
  void attitudeIGainCb(double _p);
  void headingNaturalFreqCb(double _p);
  void headingDampingRatioCb(double _p);
  void headingIGainCb(double _p);
  void throttleGainThresholdCb(double _p);

  void droneCb(const Drone::ConstSharedPtr& _drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& _tree);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& _dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& _js);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& _landed);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& _rotor_liveliness);
  void positionCommandCb(const tobas_command_msgs::PosVelAcc::ConstSharedPtr& _pos_cmd);
  void accelCommandCb(const tobas_command_msgs::Accel::ConstSharedPtr& _acc_cmd);
  void angleCommandCb(const tobas_command_msgs::Angle::ConstSharedPtr& _angle_cmd);
  void rateCommandCb(const tobas_command_msgs::Rate::ConstSharedPtr& _rate_cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& _options)
  : super(node::kController, nodeOptions_DParam(_options)), js_converter_(tree_), mixer_(drone_, tree_)
{
  // Get static parameters.
  do_dist_comp_trans_ = getBoolParam("do_disturbance_compensation_translation");
  do_dist_comp_rot_ = getBoolParam("do_disturbance_compensation_rotation");

  // Register dynamic parameters.
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFreqCb, this, 0.2, 5, 1, 30, " rad/s");
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFreqCb, this, 0.2, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFreqCb, this, 1.0, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFreqCb, this, 0.5, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 0.1, 7, 1, 20);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 0.1, 10, 1, 20);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 0.1, 10, 1, 20);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 0.1, 10, 1, 20);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0.01, 10, 0, 30);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0.01, 10, 0, 30);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0.1, 10, 0, 30);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0.01, 10, 0, 30);
  addDynamicDoubleParam("horizontal_i_max_accel", &self::horizontalIMaxAccelCb, this, 0.5, 4, 0, 20, " m/s^2");
  addDynamicDoubleParam("vertical_i_max_accel", &self::verticalIMaxAccelCb, this, 0.5, 4, 0, 20, " m/s^2");
  addDynamicDoubleParam("throttle_gain_threshold", &self::throttleGainThresholdCb, this, 1.0, 70, 0, 100, " %");

  // Register publishers.
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(topic::kRotorThrustsCmd);
  setpoint_pub_ = createPublisher<tobas_msgs::OdometryStamped>(topic::kTrajSetpoint);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MulticopterControllerFeedback>(topic::kMRCtrlFeedback);

  // Register subscribers.
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
  if (do_dist_comp_trans_ || do_dist_comp_rot_) {
    dist_force_sub_ = createSubscriber(topic::kDisturbanceForce, &self::disturbanceForceCb, this);
  }
  landed_sub_ = createSubscriber(topic::kLanded, &self::landedCb, this);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  rotor_liveliness_sub_ = createSubscriber(topic::kRotorLiv, &self::rotorLivelinessCb, this);
  pos_cmd_sub_ = createSubscriber(topic::kPosVelAccCmd, &self::positionCommandCb, this);
  acc_cmd_sub_ = createSubscriber(topic::kAccelCmd, &self::accelCommandCb, this);
  angle_cmd_sub_ = createSubscriber(topic::kAngleCmd, &self::angleCommandCb, this);
  rate_cmd_sub_ = createSubscriber(topic::kRateCmd, &self::rateCommandCb, this);

  // Register timers.
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

void ControllerNode::updateInternalDataStructures()
{
  js_converter_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();
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
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return false;
  }

  return true;
}

void ControllerNode::horizontalNaturalFreqCb(double _p)
{
  trans_ctrl_.hor_wn = _p;
}

void ControllerNode::horizontalDampingRatioCb(double _p)
{
  trans_ctrl_.hor_zeta = _p;
}

void ControllerNode::horizontalIGainCb(double _p)
{
  trans_ctrl_.hor_ki = _p;
}

void ControllerNode::horizontalIMaxAccelCb(double _p)
{
  trans_ctrl_.hor_max_i_acc = _p;
}

void ControllerNode::verticalNaturalFreqCb(double _p)
{
  trans_ctrl_.ver_wn = _p;
}

void ControllerNode::verticalDampingRatioCb(double _p)
{
  trans_ctrl_.ver_zeta = _p;
}

void ControllerNode::verticalIGainCb(double _p)
{
  trans_ctrl_.ver_ki = _p;
}

void ControllerNode::verticalIMaxAccelCb(double _p)
{
  trans_ctrl_.ver_max_i_acc = _p;
}

void ControllerNode::attitudeNaturalFreqCb(double _p)
{
  rot_ctrl_.atti_wn = _p;
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
  rot_ctrl_.head_wn = _p;
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
  drone_received_ = true;

  if (!js_sub_ && _drone->hasServoJoint()) {
    js_sub_ = createSubscriber(topic::kJointStates, &self::jointStateCb, this);
  }
  else if (js_sub_ && !_drone->hasServoJoint()) {
    js_sub_.reset();
  }

  if (drone_received_ && tree_received_) {
    updateInternalDataStructures();
  }
}

void ControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& _tree)
{
  tree_ = *_tree;
  tree_received_ = true;

  if (drone_received_ && tree_received_) {
    updateInternalDataStructures();
  }
}

void ControllerNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _odom)
{
  if (!odom_) {
    odom_ = _odom;
    return;
  }

  // Compute elapsed time and update odometry.
  const auto& cur_time = _odom->header.stamp;
  const auto dt = (cur_time - odom_->header.stamp).seconds();
  odom_ = _odom;

  // Create the setpoint message.
  auto setpoint = std::make_unique<tobas_msgs::OdometryStamped>();
  setpoint->header.stamp = cur_time;
  setpoint->odom.setNaN();

  // Create the feedback message.
  auto feedback = std::make_unique<tobas_debug_msgs::MulticopterControllerFeedback>();
  feedback->header.stamp = cur_time;

  // Aliases.
  const auto& cur_pos_W = _odom->odom.odom.frame.p;
  const auto& cur_rot = _odom->odom.odom.frame.M;
  const auto& cur_vel_B = _odom->odom.odom.twist.vel;
  const auto& cur_gyro_B = _odom->odom.odom.twist.rot;

  // Gain scheduling.
  const auto tar_sf = acc_cmd_ ? std::max(acc_cmd_->accel.z() + st::kGravity, 0.0) : 0.0;
  const auto sf_thresh = st::kGravity * throttle_gain_thresh_;
  const auto land_suspect = (tar_sf < sf_thresh);
  const auto gain_throt = land_suspect ? tar_sf / sf_thresh : 1.0;

  // Position controller.
  if (pos_cmd_) {
    if (!acc_cmd_) {
      acc_cmd_.emplace();
    }

    // Determine gains.
    const auto hor_wn = trans_ctrl_.hor_wn * gain_throt;
    const auto ver_wn = trans_ctrl_.ver_wn;  // Do not throttle the vertical direction.
    const auto hor_kp = math::sqr(hor_wn);
    const auto ver_kp = math::sqr(ver_wn);
    const auto hor_ki = trans_ctrl_.hor_ki * gain_throt;
    const auto ver_ki = trans_ctrl_.ver_ki;  // Do not throttle the vertical direction.
    const auto hor_kd = 2 * trans_ctrl_.hor_zeta * hor_wn;
    const auto ver_kd = 2 * trans_ctrl_.ver_zeta * ver_wn;
    const kdl::Vector kp(hor_kp, hor_kp, ver_kp);
    const kdl::Vector ki(hor_ki, hor_ki, ver_ki);
    const kdl::Vector kd(hor_kd, hor_kd, ver_kd);

    // Compute error.
    const auto ep = pos_cmd_->pos - cur_pos_W;
    const auto ed = pos_cmd_->vel - cur_rot * cur_vel_B;

    // Accumulate integral error while airborne.
    if (!land_suspect) {
      assert(trans_ctrl_.hor_ki > 0.0);
      assert(trans_ctrl_.ver_ki > 0.0);
      const auto hor_max_ei = trans_ctrl_.hor_max_i_acc / trans_ctrl_.hor_ki;
      const auto ver_max_ei = trans_ctrl_.ver_max_i_acc / trans_ctrl_.ver_ki;
      const kdl::Vector max_ei(hor_max_ei, hor_max_ei, ver_max_ei);
      const auto next_ei = trans_ctrl_.ei + ep * dt;
      trans_ctrl_.ei = next_ei.clamp(-max_ei, max_ei);
    }

    // Compute target acceleration.
    acc_cmd_->accel = pos_cmd_->acc + kp.hadamard(ep) + ki.hadamard(trans_ctrl_.ei) + kd.hadamard(ed);

    // Fill the feedback message.
    setpoint->odom.frame.p = pos_cmd_->pos;
    setpoint->odom.twist.vel = cur_rot.inverse(pos_cmd_->vel);
    feedback->position_integral_error = trans_ctrl_.ei;
  }

  // Attitude controller.
  if (angle_cmd_) {
    if (!rate_cmd_) {
      rate_cmd_.emplace();
    }

    // Determine gains.
    const auto atti_wn = rot_ctrl_.atti_wn * gain_throt;
    const auto head_wn = rot_ctrl_.head_wn * gain_throt;
    const auto atti_angle_gain = atti_wn / rot_ctrl_.atti_zeta / 2;
    const auto head_angle_gain = head_wn / rot_ctrl_.head_zeta / 2;
    const kdl::Vector angle_gain(atti_angle_gain, atti_angle_gain, head_angle_gain);
    const auto atti_ki = rot_ctrl_.atti_ki * gain_throt;
    const auto head_ki = rot_ctrl_.head_ki * gain_throt;
    const kdl::Vector ki(atti_ki, atti_ki, head_ki);

    // Compute rotation error.
    const auto tar_rot = angle_cmd_->angle.toRotation();
    const auto ep = (cur_rot.inverse() * tar_rot).getRot();

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

    // Compute target angular velocity.
    rate_cmd_->rate = angle_gain.hadamard(ep) + ki.hadamard(rot_ctrl_.ei);

    // Fill the feedback message.
    setpoint->odom.frame.M = tar_rot;
    feedback->angle_integral_error = rot_ctrl_.ei;
  }

  // Angular velocity controller.
  if (rate_cmd_) {
    if (!tar_dgyro_) {
      tar_dgyro_.emplace();
    }

    // Determine gains.
    const auto atti_wn = rot_ctrl_.atti_wn * gain_throt;
    const auto head_wn = rot_ctrl_.head_wn * gain_throt;
    const auto atti_rate_gain = atti_wn * rot_ctrl_.atti_zeta * 2;
    const auto head_rate_gain = head_wn * rot_ctrl_.head_zeta * 2;
    const kdl::Vector rate_gain(atti_rate_gain, atti_rate_gain, head_rate_gain);

    // Compute target angular acceleration.
    *tar_dgyro_ = rate_gain.hadamard(rate_cmd_->rate - cur_gyro_B);

    // Fill the feedback message.
    setpoint->odom.twist.rot = rate_cmd_->rate;
  }

  // Mixer.
  if (acc_cmd_ && tar_dgyro_) {
    // Convert 6-axis acceleration to propeller thrust.
    const auto& dist_force_W = do_dist_comp_trans_ ? dist_force_->wrench.force : kdl::Vector::Zero();
    const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
    const auto result = mixer_.solve(
      js_converter_.getPosition(), cur_rot, cur_gyro_B, acc_cmd_->accel, *tar_dgyro_, dist_force_W, dist_torque_B);
    if (!result) {
      TOBAS_FATAL("Failed to solve the mixing equation: ", result.error());
      // TODO: Defensive behavior
      return;
    }

    // Publish target thrust.
    auto tar_thrusts = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    tar_thrusts->header.stamp = cur_time;
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      tar_thrusts->thrusts.emplace_back();
      tar_thrusts->thrusts.back().link_name = rotor_it.first;
      tar_thrusts->thrusts.back().thrust = (*result)(idx);
    }
    tar_thrusts_pub_->publish(std::move(tar_thrusts));

    // Fill the feedback message.
    setpoint->odom.accel.linear = cur_rot.inverse(acc_cmd_->accel);
    setpoint->odom.accel.angular = *tar_dgyro_;

    // Publish the feedback message.
    setpoint_pub_->publish(std::move(setpoint));
    feedback_pub_->publish(std::move(feedback));
  }
}

void ControllerNode::disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& _dist_force)
{
  dist_force_ = _dist_force;
}

void ControllerNode::jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& _js)
{
  // Assume that information for different joints may arrive in separate messages, and convert to KDL inside the
  // callback instead of storing the message itself.
  if (const auto result = js_converter_.convert(*_js); !result) {
    TOBAS_ERROR("Joint state converter failed: ", result.error());
    return;
  }

  js_received_ = true;
}

void ControllerNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& _landed)
{
  landed_ = _landed;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming)
{
  if (!arming_) {
    arming_ = _arming;
    return;
  }

  // Reset integral errors and commands when disarmed.
  if (!_arming->data && arming_->data) {
    trans_ctrl_.ei.setZero();
    rot_ctrl_.ei.setZero();

    pos_cmd_.reset();
    acc_cmd_.reset();
    angle_cmd_.reset();
    rate_cmd_.reset();
    tar_dgyro_.reset();

    TOBAS_INFO("The controller has been reset.");
  }

  arming_ = _arming;
}

void ControllerNode::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& _rotor_liveliness)
{
  if (!mixer_.isInitialized()) {
    return;
  }

  for (const auto& data : _rotor_liveliness->data) {
    if (const auto result = mixer_.setRotorLiveliness(data.link_name, data.alive); !result) {
      TOBAS_ERROR("Failed to set rotor liveliness: ", result.error());
    }
  }
}

void ControllerNode::positionCommandCb(const tobas_command_msgs::PosVelAcc::ConstSharedPtr& _pos_cmd)
{
  if (!isCommandAccepted(_pos_cmd->priority)) {
    return;
  }

  // Create the command.
  if (!pos_cmd_) {
    pos_cmd_.emplace();
  }

  // Update the command.
  *pos_cmd_ = *_pos_cmd;
}

void ControllerNode::accelCommandCb(const tobas_command_msgs::Accel::ConstSharedPtr& _acc_cmd)
{
  if (!isCommandAccepted(_acc_cmd->priority)) {
    return;
  }

  // Stop the outer control loop.
  pos_cmd_.reset();

  // Create the command.
  if (!acc_cmd_) {
    acc_cmd_.emplace();
  }

  // Update the command.
  *acc_cmd_ = *_acc_cmd;
}

void ControllerNode::angleCommandCb(const tobas_command_msgs::Angle::ConstSharedPtr& _angle_cmd)
{
  if (!isCommandAccepted(_angle_cmd->priority)) {
    return;
  }

  // Create the command.
  if (!angle_cmd_) {
    angle_cmd_.emplace();
  }

  // Update the command.
  *angle_cmd_ = *_angle_cmd;
}

void ControllerNode::rateCommandCb(const tobas_command_msgs::Rate::ConstSharedPtr& _rate_cmd)
{
  if (!isCommandAccepted(_rate_cmd->priority)) {
    return;
  }

  // Stop the outer control loop.
  angle_cmd_.reset();

  // Create the command.
  if (!rate_cmd_) {
    rate_cmd_.emplace();
  }

  // Update the command.
  *rate_cmd_ = *_rate_cmd;
}

void ControllerNode::checkTopicsTimerCb()
{
  if (!drone_received_) {
    TOBAS_WARN("Waiting for '", topic::kDrone, "'.");
    return;
  }

  if (!tree_received_) {
    TOBAS_WARN("Waiting for '", topic::kKdlTree, "'.");
    return;
  }

  if (!odom_) {
    TOBAS_WARN("Waiting for '", topic::kOdometry, "'.");
    return;
  }

  if (dist_force_sub_ && !dist_force_) {
    TOBAS_WARN("Waiting for '", topic::kDisturbanceForce, "'.");
    return;
  }

  if (js_sub_ && !js_received_) {
    TOBAS_WARN("Waiting for '", topic::kJointStates, "'.");
    return;
  }

  if (!landed_) {
    TOBAS_WARN("Waiting for '", topic::kLanded, "'.");
    return;
  }

  if (!arming_) {
    TOBAS_WARN("Waiting for '", topic::kArming, "'.");
    return;
  }

  topics_received_ = true;
  check_topics_timer_->cancel();
}
}  // namespace tobas::nonplanar_multicopter

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::nonplanar_multicopter::ControllerNode)
