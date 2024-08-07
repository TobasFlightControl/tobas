#pragma once

#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_kdl/treemassholder.hpp>
#include <tobas_linear_control/lqd.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_tools/rotor_axis_extractor.hpp>
#include <tobas_drone_tools/fw_micro_disturbance_eom.hpp>
#include <tobas_drone_tools/utils/fixed_wing_tools.hpp>
#include <tobas_tools/conversions/coordinates.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/control_surface_deflections.hpp>
#include <tobas_msgs/msg/fixed_wing_controller_feedback.hpp>
#include <tobas_msgs/Odometry.hpp>

using namespace std;
using namespace Eigen;

namespace tobas_fixed_wing_lqd
{
class ControllerNode : public tobas::BaseNode
{
  using self = ControllerNode;
  using super = tobas::BaseNode;

public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  kdl::Tree tree_;

  kdl::TreeMassHolder mass_holder_;
  tobas::RotorAxisExtractor x_rotors_;
  tobas::MicroDisturbanceEoM eom_;  // 微小擾乱状態方程式

  // 固定値
  kdl::JntArray q_0_;

  double cur_roll_, cur_pitch_, cur_yaw_;
  sensor_msgs::msg::FluidPressure::ConstSharedPtr air_pressure_;  // 大気圧
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;              // 現在のバッテリーの状態
  tobas_msgs::Odometry::ConstSharedPtr odom_nwu_;                 // 現在の状態 (NWU座標系)
  tobas_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr cmd_nwu_;  // 現在のコマンド (NWU座標系)
  tobas_msgs::Odometry odom_ned_;                                 // 現在の状態 (NED座標系)
  std_msgs::msg::Bool::ConstSharedPtr arming_;                    // ロータのアーム状態
  tobas_msgs::msg::SpeedRollDeltaPitch cmd_ned_;                  // 現在のコマンド (NED座標系)

  ctrl::LQD lqd_;  // 最適レギュレータ

  // Publishers
  PublisherPtr<tobas_msgs::msg::RotorSpeeds> rot_speeds_pub_;
  PublisherPtr<tobas_msgs::msg::ControlSurfaceDeflections> deflections_pub_;
  PublisherPtr<tobas_msgs::msg::FixedWingControllerFeedback> feedback_pub_;

  // Subscribers
  SubscriberPtr<sensor_msgs::msg::FluidPressure> air_pressure_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  SubscriberPtr<tobas_msgs::msg::SpeedRollDeltaPitch> cmd_sub_;

  bool isReadyToControl();
  void setScales();
  void updateCurrentStateVector();
  void updateSetStateVector();
  void publishRotSpeeds(const Eigen::VectorXd& thrust);
  void publishDeflections(const Eigen::VectorXd& deflections);
  void publishFeedback(const Eigen::VectorXd& du);

  void airPressureCb(const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom_nwu);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void commandCb(const tobas_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr& cmd_nwu);

  bool forwardSpeedWeightCb(const long& p);
  bool alphaWeightCb(const long& p);
  bool betaWeightCb(const long& p);
  bool attitudeWeightCb(const long& p);
  bool angularVelicityWeightCb(const long& p);
  bool thrustWeightLog10Cb(const long& p);
  bool thrustRateWeightLog10Cb(const long& p);
  bool deflectionWeightLog10Cb(const long& p);
  bool deflectionRateWeightLog10Cb(const long& p);
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::kControllerNode, options),
    mass_holder_(tree_),
    x_rotors_(drone_, tobas::X_POSITIVE),
    eom_(drone_, tree_)
{
  // TODO: Drone, Treeを取得

  // FixedWingを持つことを確認

  mass_holder_.updateInternalDataStructures();
  x_rotors_.updateInternalDataStructures();
  eom_.updateInternalDataStructures();

  if (x_rotors_.count() == 0)
    TOBAS_EXIT("The number of propellers is zero.");

  q_0_.resize(tree_.getNrOfJoints());
  q_0_.setZero();

  setScales();
  lqd_.state_weight.resize(eom_.kStateSize);
  lqd_.input_weight.resize(eom_.inputSize());
  lqd_.input_rate_weight.resize(eom_.inputSize());
  lqd_.current_state.resize(eom_.kStateSize);
  lqd_.target_state.resize(eom_.kStateSize);
  lqd_.last_input = VectorXd::Zero(eom_.inputSize());

  // Register publishers
  rot_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic);
  deflections_pub_ = createPublisher<tobas_msgs::msg::ControlSurfaceDeflections>(tobas::kDeflectionCmdTopic);
  feedback_pub_ = createPublisher<tobas_msgs::msg::FixedWingControllerFeedback>(tobas::kControllerFeedbackTopic);

  // Register subscribers
  air_pressure_sub_ = createSubscriber(tobas::kAirPressureTopic, &self::airPressureCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryLpfTopic, &self::batteryCb, this);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  cmd_sub_ = createSubscriber(tobas::kSpeedRollDpitchCmdTopic, &self::commandCb, this);

  // Register dynamic parameters
  addDynamicIntParam("forward_speed_weight", &self::forwardSpeedWeightCb, this, 1, 1, 100);

  publishDynamicParameterDescriptions();
}

bool ControllerNode::isReadyToControl()
{
  if (air_pressure_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kAirPressureTopic);
    return false;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kBatteryLpfTopic);
    return false;
  }

  if (odom_nwu_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kOdometryTopic);
    return false;
  }

  if (odom_nwu_->status != tobas_msgs::msg::Odometry::NO_ERROR)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "There is a problem with the state estimation.");
    return false;
  }

  if (arming_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kArmingTopic);
    return false;
  }

  if (!arming_->data)
    return false;

  return true;
}

void ControllerNode::setScales()
{
  // 状態変数のスケール
  lqd_.state_scale.resize(eom_.kStateSize);
  lqd_.state_scale(eom_.kStateIdx_u) = eom_.trimCondition().takeOffSpeed(tobas_std::kStandardAirDensity);
  lqd_.state_scale(eom_.kStateIdx_alpha) = drone_.fixed_wing.vehicle.alpha_limit.range();
  lqd_.state_scale(eom_.kStateIdx_beta) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_phi) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_theta) = M_PI_4;
  lqd_.state_scale(eom_.kStateIdx_p) = M_PI;
  lqd_.state_scale(eom_.kStateIdx_q) = M_PI;
  lqd_.state_scale(eom_.kStateIdx_r) = M_PI;

  // 制御入力のスケール
  lqd_.input_scale.resize(eom_.inputSize());
  const auto thrust_scale = mass_holder_.getMass() * tobas_std::kGravity / x_rotors_.count();
  lqd_.input_scale.block(0, 0, x_rotors_.count(), 1).fill(thrust_scale);
  for (size_t i = 0; i < drone_.fixed_wing.control_surfaces.size(); ++i)
    lqd_.input_scale(x_rotors_.count() + i) = drone_.fixed_wing.control_surfaces.at(i).angle_limit.range();
}

void ControllerNode::updateCurrentStateVector()
{
  const auto& trim = eom_.trimCondition();
  odom_ned_.frame.M.getRPY(cur_roll_, cur_pitch_, cur_yaw_);

  // TODO: 横系のトリムも考慮
  lqd_.current_state(eom_.kStateIdx_u) = odom_ned_.twist.vel.x() - trim.u();
  lqd_.current_state(eom_.kStateIdx_alpha) = tobas::angleOfAttack(odom_ned_.twist.vel.data) - trim.alpha();
  lqd_.current_state(eom_.kStateIdx_beta) = tobas::angleOfSideSlip(odom_ned_.twist.vel.data);
  lqd_.current_state(eom_.kStateIdx_phi) = cur_roll_;
  lqd_.current_state(eom_.kStateIdx_theta) = cur_pitch_ - trim.theta();
  lqd_.current_state(eom_.kStateIdx_p) = odom_ned_.twist.rot.x();
  lqd_.current_state(eom_.kStateIdx_q) = odom_ned_.twist.rot.y();
  lqd_.current_state(eom_.kStateIdx_r) = odom_ned_.twist.rot.z();
}

void ControllerNode::updateSetStateVector()
{
  const auto& trim = eom_.trimCondition();

  // 失速しないように速度制限をした上で目標推力を計算
  const auto rho = tobas_std::pressureToDensity(air_pressure_->fluid_pressure);
  const auto tar_speed = trim.speedLimit(rho).clamp(cmd_ned_.speed);
  const auto tar_u = tar_speed * cos(eom_.trimCondition().alpha());

  lqd_.target_state(eom_.kStateIdx_u) = tar_u - trim.u();
  lqd_.target_state(eom_.kStateIdx_alpha) = 0.;
  lqd_.target_state(eom_.kStateIdx_beta) = 0.;
  lqd_.target_state(eom_.kStateIdx_phi) = cmd_ned_.roll;
  lqd_.target_state(eom_.kStateIdx_theta) = cmd_ned_.delta_pitch;
  lqd_.target_state(eom_.kStateIdx_p) = 0.;
  lqd_.target_state(eom_.kStateIdx_q) = 0.;
  lqd_.target_state(eom_.kStateIdx_r) = 0.;
}

void ControllerNode::publishRotSpeeds(const VectorXd& thrust)
{
  auto rot_speeds = std::make_unique<tobas_msgs::msg::RotorSpeeds>();
  rot_speeds->header.stamp = odom_ned_.header.stamp;

  rot_speeds->speeds.resize(drone_.rotors.size(), 0.);
  for (size_t i = 0; i < static_cast<size_t>(thrust.rows()); ++i)
    rot_speeds->speeds[x_rotors_.rotorIdx(i)] = x_rotors_.rotSpeedFromThrust(i, max(0., thrust(i)));

  rot_speeds_pub_->publish(move(rot_speeds));
}

void ControllerNode::publishDeflections(const VectorXd& deflections)
{
  auto deflections_msg = std::make_unique<tobas_msgs::msg::ControlSurfaceDeflections>();
  deflections_msg->header.stamp = odom_ned_.header.stamp;
  deflections_msg->deflections = eigen_tools::toStdVector(deflections);
  deflections_pub_->publish(move(deflections_msg));
}

void ControllerNode::publishFeedback(const VectorXd& du)
{
  const auto& trim = eom_.trimCondition();
  auto feedback = std::make_unique<tobas_msgs::msg::FixedWingControllerFeedback>();

  feedback->trim_thrusts.resize(drone_.rotors.size());
  feedback->delta_thrusts.resize(drone_.rotors.size());
  feedback->trim_deflections.resize(drone_.fixed_wing.control_surfaces.size());
  feedback->delta_deflections.resize(drone_.fixed_wing.control_surfaces.size());

  feedback->trim_u = trim.u();
  feedback->trim_alpha = trim.alpha();

  for (size_t i = 0; i < x_rotors_.count(); ++i)
  {
    feedback->trim_thrusts[x_rotors_.rotorIdx(i)] = eom_.trimInput()(i);
    feedback->delta_thrusts[x_rotors_.rotorIdx(i)] = du(i);
  }

  for (size_t i = 0; i < drone_.fixed_wing.control_surfaces.size(); ++i)
  {
    const auto u_idx = x_rotors_.count() + i;
    feedback->trim_deflections[i] = eom_.trimInput()[u_idx];
    feedback->delta_deflections[i] = du(u_idx);
  }

  feedback_pub_->publish(move(feedback));
}

void ControllerNode::airPressureCb(const sensor_msgs::msg::FluidPressure::ConstSharedPtr& msg)
{
  air_pressure_ = msg;
}

void ControllerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void ControllerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom_nwu)
{
  if (odom_nwu_ == nullptr)
  {
    odom_nwu_ = odom_nwu;
    return;
  }

  // 経過時間を計算してオドメトリを更新
  const auto dt = (odom_nwu->header.stamp - odom_nwu_->header.stamp).seconds();
  odom_nwu_ = odom_nwu;

  if (!isReadyToControl())
    return;

  // コマンドが来ていなければスキップ
  if (cmd_nwu_ == nullptr)
    return;

  // NWU -> NED
  tobas::odometryNwuToNed(*odom_nwu_, odom_ned_);
  tobas::speedRollDeltaPitchNwuToNed(*cmd_nwu_, cmd_ned_);

  // 現在の速度を使って状態方程式を更新
  const auto rho = tobas_std::pressureToDensity(air_pressure_->fluid_pressure);
  switch (eom_.update(odom_ned_.twist.vel.norm(), rho, battery_->voltage, q_0_))
  {
    case tobas::SolverI::E_NO_ERROR:
      break;
    case tobas::SolverI::E_WARN:
      TOBAS_WARN(eom_.errorMessage());
      break;
    case tobas::SolverI::E_ERROR:
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
  const VectorXd du = lqd_.solve(dt);
  const VectorXd u = eom_.trimInput() + du;

  const VectorXd thrust = u.block(0, 0, x_rotors_.count(), 1);
  const VectorXd deflections = u.block(x_rotors_.count(), 0, drone_.fixed_wing.control_surfaces.size(), 1);

  // Publish
  publishRotSpeeds(thrust);
  publishDeflections(deflections);
  publishFeedback(du);
}

void ControllerNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (!arming->data)
  {
    cmd_nwu_ = nullptr;
    lqd_.last_input.setZero();
    TOBAS_INFO("ControllerNode is reset.");
  }
}

void ControllerNode::commandCb(const tobas_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr& cmd_nwu)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  // TODO: コマンドレベルの処理

  cmd_nwu_ = cmd_nwu;
}

bool ControllerNode::forwardSpeedWeightCb(const long& p)
{
  lqd_.state_weight(eom_.kStateIdx_u) = p;
  return true;
}

bool ControllerNode::alphaWeightCb(const long& p)
{
  // TODO
}

bool ControllerNode::betaWeightCb(const long& p)
{
  // TODO
}

bool ControllerNode::attitudeWeightCb(const long& p)
{
  // TODO
}

bool ControllerNode::angularVelicityWeightCb(const long& p)
{
  // TODO
}

bool ControllerNode::thrustWeightLog10Cb(const long& p)
{
  // TODO
}

bool ControllerNode::thrustRateWeightLog10Cb(const long& p)
{
  // TODO
}

bool ControllerNode::deflectionWeightLog10Cb(const long& p)
{
  // TODO
}

bool ControllerNode::deflectionRateWeightLog10Cb(const long& p)
{
  // TODO
}
}  // namespace tobas_fixed_wing_lqd
