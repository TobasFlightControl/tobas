// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <ranges>

#include <tobas_constants/node.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_control/lqd.hpp>
#include <tobas_drone_tools/fw_micro_disturbance_eom.hpp>
#include <tobas_drone_tools/utils/fixed_wing_tools.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/command_priority_handler.hpp>
#include <tobas_tools/coordinates.hpp>

#include <tobas_command_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

namespace tobas
{
namespace fixed_wing
{
struct ControllerParameters
{
  long forward_speed_weight;
  long alpha_weight;
  long beta_weight;
  long attitude_weight;
  long angular_velocity_weight;
  long thrust_weight_log10;
  long thrust_rate_weight_log10;
  long deflection_weight_log10;
  long deflection_rate_weight_log10;
};

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
  MicroDisturbanceEoM eom_;  // 微小擾乱状態方程式

  // 固定値
  kdl::JntArray q_0_;

  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool topics_received_ = false;
  CommandPriorityHandler cmd_priority_handler_;
  tobas_msgs::msg::FluidPressure::ConstSharedPtr air_pressure_;           // 大気圧
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_flu_;    // 現在の状態 (FLU座標系)
  tobas_command_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr cmd_flu_;  // 現在のコマンド (FLU座標系)
  tobas_msgs::Odometry odom_frd_;                                         // 現在の状態 (FRD座標系)
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;                        // ロータのアーム状態
  tobas_command_msgs::msg::SpeedRollDeltaPitch cmd_frd_;                  // 現在のコマンド (FRD座標系)
  ControllerParameters params_;
  ctrl::LQD lqd_;  // 最適レギュレータ

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> tar_angles_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressure> air_pressure_sub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::SpeedRollDeltaPitch> cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool initialize();
  void updateCurrentStateVector();
  void updateSetStateVector();
  void publishThrusts(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& thrusts);
  void publishDeflections(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& deflections);
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& priority);

  void updateForwardSpeedWeight();
  void updateAlphaWeight();
  void updateBetaWeight();
  void updateAttitudeWeight();
  void updateAngularVelicityWeight();
  void updateThrustWeightLog10();
  void updateThrustRateWeightLog10();
  void updateDeflectionWeightLog10();
  void updateDeflectionRateWeightLog10();
  void updateParameters();

  bool forwardSpeedWeightCb(const long& p);
  bool alphaWeightCb(const long& p);
  bool betaWeightCb(const long& p);
  bool attitudeWeightCb(const long& p);
  bool angularVelicityWeightCb(const long& p);
  bool thrustWeightLog10Cb(const long& p);
  bool thrustRateWeightLog10Cb(const long& p);
  bool deflectionWeightLog10Cb(const long& p);
  bool deflectionRateWeightLog10Cb(const long& p);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void airPressureCb(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& pressure);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom_flu);
  void commandCb(const tobas_command_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr& cmd_flu);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(node::kController, nodeOptions_DParam(options)), mass_holder_(tree_), eom_(drone_, tree_)
{
  // Register dynamic parameters
  addDynamicIntParam("forward_speed_weight", &self::forwardSpeedWeightCb, this, 1, 1, 100);
  addDynamicIntParam("alpha_weight", &self::alphaWeightCb, this, 1, 1, 100);
  addDynamicIntParam("beta_weight", &self::betaWeightCb, this, 1, 1, 100);
  addDynamicIntParam("attitude_weight", &self::attitudeWeightCb, this, 1, 1, 100);
  addDynamicIntParam("angular_velocity_weight", &self::angularVelicityWeightCb, this, 1, 1, 100);
  addDynamicIntParam("thrust_weight_log10", &self::thrustWeightLog10Cb, this, -3, -3, 3);
  addDynamicIntParam("thrust_rate_weight_log10", &self::thrustRateWeightLog10Cb, this, -1, -3, 3);
  addDynamicIntParam("deflection_weight_log10", &self::deflectionWeightLog10Cb, this, -3, -3, 3);
  addDynamicIntParam("deflection_rate_weight_log10", &self::deflectionRateWeightLog10Cb, this, -1, -3, 3);

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(topic::kRotorThrustsCmd);
  tar_angles_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(topic::kJointPosCmd);

  // Register subscribers
  drone_sub_ = createSubscriber(topic::kDrone, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(topic::kKdlTree, &self::treeCb, this, true, true);
  arming_sub_ = createSubscriber(topic::kArming, &self::armingCb, this);
  air_pressure_sub_ = createSubscriber(topic::kAirPressure, &self::airPressureCb, this);
  odom_sub_ = createSubscriber(topic::kOdometry, &self::odomCb, this);
  cmd_sub_ = createSubscriber(topic::kSpeedRollDpitchCmd, &self::commandCb, this);

  // Register timers
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::initialize()
{
  if (!mass_holder_.updateInternalDataStructures()) {
    return false;
  }
  if (!eom_.updateInternalDataStructures()) {
    return false;
  }

  q_0_.resize(tree_.getNrOfJoints());
  q_0_.setZero();

  // 状態変数のスケール
  lqd_.state_scale.resize(eom_.kStateSize);
  lqd_.state_scale(eom_.kStateIdx_u) = eom_.trimCondition().takeOffSpeed(st::kStandardAirDensity);
  lqd_.state_scale(eom_.kStateIdx_alpha) = drone_.fixed_wing->vehicle.alpha_limit.range();
  lqd_.state_scale(eom_.kStateIdx_beta) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_phi) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_theta) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_p) = M_PI;
  lqd_.state_scale(eom_.kStateIdx_q) = M_PI;
  lqd_.state_scale(eom_.kStateIdx_r) = M_PI;

  // 制御入力のスケール
  lqd_.input_scale.resize(eom_.inputSize());
  const auto thrust_scale = mass_holder_.getMass() * st::kGravity / drone_.prop->numRotors();
  lqd_.input_scale.head(drone_.prop->numRotors()).fill(thrust_scale);
  lqd_.input_scale.tail(drone_.fixed_wing->numControlSurfaces()).fill(M_PI);

  lqd_.state_weight.resize(eom_.kStateSize);
  lqd_.input_weight.resize(eom_.inputSize());
  lqd_.input_rate_weight.resize(eom_.inputSize());
  lqd_.current_state.resize(eom_.kStateSize);
  lqd_.target_state.resize(eom_.kStateSize);
  lqd_.last_input = Eigen::VectorXd::Zero(eom_.inputSize());

  updateParameters();

  is_initialized_ = true;
  return true;
}

void ControllerNode::updateCurrentStateVector()
{
  const auto& trim = eom_.trimCondition();
  const auto [roll, pitch, _] = odom_frd_.frame.M.getRPY();

  // TODO: 横系のトリムも考慮
  lqd_.current_state(eom_.kStateIdx_u) = odom_frd_.twist.vel.x() - trim.u();
  lqd_.current_state(eom_.kStateIdx_alpha) = angleOfAttack(odom_frd_.twist.vel.data) - trim.alpha();
  lqd_.current_state(eom_.kStateIdx_beta) = angleOfSideSlip(odom_frd_.twist.vel.data);
  lqd_.current_state(eom_.kStateIdx_phi) = roll;
  lqd_.current_state(eom_.kStateIdx_theta) = pitch - trim.theta();
  lqd_.current_state(eom_.kStateIdx_p) = odom_frd_.twist.rot.x();
  lqd_.current_state(eom_.kStateIdx_q) = odom_frd_.twist.rot.y();
  lqd_.current_state(eom_.kStateIdx_r) = odom_frd_.twist.rot.z();
}

void ControllerNode::updateSetStateVector()
{
  const auto& trim = eom_.trimCondition();

  // 失速しないように速度制限をした上で目標推力を計算
  const auto rho = st::pressureToDensity(air_pressure_->pressure);
  const auto tar_speed = trim.speedLimit(rho).clamp(cmd_frd_.speed);
  const auto tar_u = tar_speed * cos(eom_.trimCondition().alpha());

  lqd_.target_state(eom_.kStateIdx_u) = tar_u - trim.u();
  lqd_.target_state(eom_.kStateIdx_alpha) = 0.;
  lqd_.target_state(eom_.kStateIdx_beta) = 0.;
  lqd_.target_state(eom_.kStateIdx_phi) = cmd_frd_.roll;
  lqd_.target_state(eom_.kStateIdx_theta) = cmd_frd_.delta_pitch;
  lqd_.target_state(eom_.kStateIdx_p) = 0.;
  lqd_.target_state(eom_.kStateIdx_q) = 0.;
  lqd_.target_state(eom_.kStateIdx_r) = 0.;
}

void ControllerNode::publishThrusts(const builtin_interfaces::msg::Time& stamp, const Eigen::VectorXd& thrusts)
{
  assert(static_cast<size_t>(thrusts.size()) == drone_.prop->numRotors());

  auto thrusts_msg = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
  thrusts_msg->header.stamp = stamp;

  for (const auto& [idx, elem] : std::views::enumerate(drone_.prop->rotors)) {
    thrusts_msg->thrusts.emplace_back();
    thrusts_msg->thrusts.back().link_name = elem.first;
    thrusts_msg->thrusts.back().thrust = std::max(thrusts(idx), 0.);
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
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because some topics are not received yet.");
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

void ControllerNode::updateForwardSpeedWeight()
{
  lqd_.state_weight(eom_.kStateIdx_u) = params_.forward_speed_weight;
}

void ControllerNode::updateAlphaWeight()
{
  lqd_.state_weight(eom_.kStateIdx_alpha) = params_.alpha_weight;
}

void ControllerNode::updateBetaWeight()
{
  lqd_.state_weight(eom_.kStateIdx_beta) = params_.beta_weight;
}

void ControllerNode::updateAttitudeWeight()
{
  lqd_.state_weight(eom_.kStateIdx_phi) = params_.attitude_weight;
  lqd_.state_weight(eom_.kStateIdx_theta) = params_.attitude_weight;
}

void ControllerNode::updateAngularVelicityWeight()
{
  lqd_.state_weight(eom_.kStateIdx_p) = params_.angular_velocity_weight;
  lqd_.state_weight(eom_.kStateIdx_q) = params_.angular_velocity_weight;
  lqd_.state_weight(eom_.kStateIdx_r) = params_.angular_velocity_weight;
}

void ControllerNode::updateThrustWeightLog10()
{
  const auto thrust_weight = exp10(params_.thrust_weight_log10);
  lqd_.input_weight.head(drone_.prop->numRotors()).fill(thrust_weight);
}

void ControllerNode::updateThrustRateWeightLog10()
{
  const auto thrust_rate_weight = exp10(params_.thrust_rate_weight_log10);
  lqd_.input_rate_weight.head(drone_.prop->numRotors()).fill(thrust_rate_weight);
}

void ControllerNode::updateDeflectionWeightLog10()
{
  const auto deflection_weight = exp10(params_.deflection_weight_log10);
  lqd_.input_weight.tail(drone_.fixed_wing->numControlSurfaces()).fill(deflection_weight);
}

void ControllerNode::updateDeflectionRateWeightLog10()
{
  const auto deflection_rate_weight = exp10(params_.deflection_rate_weight_log10);
  lqd_.input_rate_weight.tail(drone_.fixed_wing->numControlSurfaces()).fill(deflection_rate_weight);
}

void ControllerNode::updateParameters()
{
  updateForwardSpeedWeight();
  updateAlphaWeight();
  updateBetaWeight();
  updateAttitudeWeight();
  updateAngularVelicityWeight();
  updateThrustWeightLog10();
  updateThrustRateWeightLog10();
  updateDeflectionWeightLog10();
  updateDeflectionRateWeightLog10();
}

bool ControllerNode::forwardSpeedWeightCb(const long& p)
{
  params_.forward_speed_weight = p;
  if (is_initialized_) {
    updateForwardSpeedWeight();
  }
  return true;
}

bool ControllerNode::alphaWeightCb(const long& p)
{
  params_.alpha_weight = p;
  if (is_initialized_) {
    updateAlphaWeight();
  }
  return true;
}

bool ControllerNode::betaWeightCb(const long& p)
{
  params_.beta_weight = p;
  if (is_initialized_) {
    updateBetaWeight();
  }
  return true;
}

bool ControllerNode::attitudeWeightCb(const long& p)
{
  params_.attitude_weight = p;
  if (is_initialized_) {
    updateAttitudeWeight();
  }
  return true;
}

bool ControllerNode::angularVelicityWeightCb(const long& p)
{
  params_.angular_velocity_weight = p;
  if (is_initialized_) {
    updateAngularVelicityWeight();
  }
  return true;
}

bool ControllerNode::thrustWeightLog10Cb(const long& p)
{
  params_.thrust_weight_log10 = p;
  if (is_initialized_) {
    updateThrustWeightLog10();
  }
  return true;
}

bool ControllerNode::thrustRateWeightLog10Cb(const long& p)
{
  params_.thrust_rate_weight_log10 = p;
  if (is_initialized_) {
    updateThrustRateWeightLog10();
  }
  return true;
}

bool ControllerNode::deflectionWeightLog10Cb(const long& p)
{
  params_.deflection_weight_log10 = p;
  if (is_initialized_) {
    updateDeflectionWeightLog10();
  }
  return true;
}

bool ControllerNode::deflectionRateWeightLog10Cb(const long& p)
{
  params_.deflection_rate_weight_log10 = p;
  if (is_initialized_) {
    updateDeflectionRateWeightLog10();
  }
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
    cmd_flu_.reset();
    lqd_.last_input.setZero();
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

  // 経過時間を計算してオドメトリを更新
  const auto dt = (odom_flu->header.stamp - odom_flu_->header.stamp).seconds();
  odom_flu_ = odom_flu;

  // コマンドがなければスキップ
  if (!cmd_flu_) {
    return;
  }

  // FLU -> FRD
  odometryFluToFrd(odom_flu_->odom.odom, odom_frd_);
  speedRollDeltaPitchFluToFrd(*cmd_flu_, cmd_frd_);

  // 現在の速度を使って状態方程式を更新
  const auto rho = st::pressureToDensity(air_pressure_->pressure);
  switch (eom_.update(odom_frd_.twist.vel.norm(), rho, q_0_)) {
    case SolverI::kNoError:
      break;
    case SolverI::kWarn:
      TOBAS_WARN(eom_.errorMessage());
      break;
    case SolverI::kError:
      TOBAS_ERROR(eom_.errorMessage());
      return;
    default:
      TOBAS_WARN("Unknown error code from MicroDisturbanceEoM.");
      break;
  }

  lqd_.dynamics.A = eom_.A();
  lqd_.dynamics.B = eom_.B();

  updateCurrentStateVector();
  updateSetStateVector();

  // 最適制御入力を求める
  const Eigen::VectorXd du = lqd_.solve(dt);
  const Eigen::VectorXd u = eom_.trimInput() + du;

  const Eigen::VectorXd thrusts = u.head(drone_.prop->numRotors());
  const Eigen::VectorXd deflections = u.tail(drone_.fixed_wing->numControlSurfaces());

  // Publish
  publishThrusts(odom_flu->header.stamp, thrusts);
  publishDeflections(odom_flu->header.stamp, deflections);
}

void ControllerNode::commandCb(const tobas_command_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr& cmd_flu)
{
  if (!isCommandAccepted(cmd_flu->priority)) {
    return;
  }

  cmd_flu_ = cmd_flu;
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
