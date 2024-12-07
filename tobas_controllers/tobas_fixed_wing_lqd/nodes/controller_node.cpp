#include <std_msgs/msg/bool.hpp>

#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_std_tools/debug.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_linear_control/lqd.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/conversions/coordinates.hpp>
#include <tobas_drone_tools/rotor_axis_extractor.hpp>
#include <tobas_drone_tools/fw_micro_disturbance_eom.hpp>
#include <tobas_drone_tools/utils/fixed_wing_tools.hpp>

#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/speed_roll_delta_pitch.hpp>
#include <tobas_msgs/msg/fluid_pressure_with_variance_stamped.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/control_surface_deflections.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_debug_msgs/msg/fixed_wing_controller_feedback.hpp>

using namespace std;
using namespace Eigen;

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

  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  double cur_roll_, cur_pitch_, cur_yaw_;
  tobas_msgs::msg::FluidPressureWithVarianceStamped::ConstSharedPtr air_pressure_;  // 大気圧
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;                                // 現在のバッテリーの状態
  tobas_msgs::Odometry::ConstSharedPtr odom_nwu_;                                   // 現在の状態 (NWU座標系)
  tobas_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr cmd_nwu_;  // 現在のコマンド (NWU座標系)
  tobas_msgs::Odometry odom_ned_;                                 // 現在の状態 (NED座標系)
  std_msgs::msg::Bool::ConstSharedPtr arming_;                    // ロータのアーム状態
  tobas_msgs::msg::SpeedRollDeltaPitch cmd_ned_;                  // 現在のコマンド (NED座標系)
  ControllerParameters params_;
  ctrl::LQD lqd_;  // 最適レギュレータ

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::ControlSurfaceDeflections> deflections_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::msg::FixedWingControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::FluidPressureWithVarianceStamped> air_pressure_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::SpeedRollDeltaPitch> cmd_sub_;

  bool initialize();
  bool isReadyToControl();
  void updateCurrentStateVector();
  void updateSetStateVector();
  void publishThrusts(const Eigen::VectorXd& thrusts);
  void publishDeflections(const Eigen::VectorXd& deflections);
  void publishFeedback(const Eigen::VectorXd& du);

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

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void airPressureCb(const tobas_msgs::msg::FluidPressureWithVarianceStamped::ConstSharedPtr& pressure);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom_nwu);
  void commandCb(const tobas_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr& cmd_nwu);
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::kControllerNode, options),
    mass_holder_(tree_),
    x_rotors_(drone_, tobas::X_POSITIVE),
    eom_(drone_, tree_)
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
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(tobas::kRotorThrustsCmdTopic);
  deflections_pub_ = createPublisher<tobas_msgs::msg::ControlSurfaceDeflections>(tobas::kDeflectionCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::msg::FixedWingControllerFeedback>(tobas::kFWCtrlFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true, true);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  air_pressure_sub_ = createSubscriber(tobas::kAirPressureTopic, &self::airPressureCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryTopic, &self::batteryCb, this);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  cmd_sub_ = createSubscriber(tobas::kSpeedRollDpitchCmdTopic, &self::commandCb, this);
}

bool ControllerNode::initialize()
{
  if (!mass_holder_.updateInternalDataStructures())
    return false;
  if (!x_rotors_.updateInternalDataStructures())
    return false;
  if (!eom_.updateInternalDataStructures())
    return false;

  q_0_.resize(tree_.getNrOfJoints());
  q_0_.setZero();

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

  size_t cs_idx = 0;
  for (const auto& [_, cs] : drone_.fixed_wing.control_surfaces)
  {
    lqd_.input_scale(x_rotors_.count() + cs_idx) = cs.angle_limit.range();
    ++cs_idx;
  }

  lqd_.state_weight.resize(eom_.kStateSize);
  lqd_.input_weight.resize(eom_.inputSize());
  lqd_.input_rate_weight.resize(eom_.inputSize());
  lqd_.current_state.resize(eom_.kStateSize);
  lqd_.target_state.resize(eom_.kStateSize);
  lqd_.last_input = VectorXd::Zero(eom_.inputSize());

  updateParameters();

  is_initialized_ = true;
  return true;
}

bool ControllerNode::isReadyToControl()
{
  if (!drone_received_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kDroneTopic, "\".");
    return false;
  }

  if (!tree_received_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kKDLTreeTopic, "\".");
    return false;
  }

  if (air_pressure_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kAirPressureTopic, "\".");
    return false;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kBatteryTopic, "\".");
    return false;
  }

  if (odom_nwu_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kOdometryTopic, "\".");
    return false;
  }

  if (odom_nwu_->status != tobas_msgs::msg::Odometry::NO_ERROR)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "There is a problem with the state estimation.");
    return false;
  }

  if (arming_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kArmingTopic, "\".");
    return false;
  }

  return true;
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
  const auto rho = tobas_std::pressureToDensity(air_pressure_->pressure.pressure);
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

void ControllerNode::publishThrusts(const VectorXd& thrusts)
{
  auto thrusts_msg = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
  thrusts_msg->header.stamp = odom_ned_.header.stamp;

  for (int i = 0; i < thrusts.rows(); ++i)
  {
    thrusts_msg->thrusts.emplace_back();
    thrusts_msg->thrusts.back().channel = x_rotors_.rotor(i).channel;
    thrusts_msg->thrusts.back().thrust = thrusts(i);
  }

  tar_thrusts_pub_->publish(move(thrusts_msg));
}

void ControllerNode::publishDeflections(const VectorXd& deflections)
{
  auto deflections_msg = std::make_unique<tobas_msgs::msg::ControlSurfaceDeflections>();
  deflections_msg->header.stamp = odom_ned_.header.stamp;
  deflections_msg->deflections = eigen::toStdVector(deflections);
  deflections_pub_->publish(move(deflections_msg));
}

void ControllerNode::publishFeedback(const VectorXd& du)
{
  const auto& trim = eom_.trimCondition();
  auto feedback = std::make_unique<tobas_debug_msgs::msg::FixedWingControllerFeedback>();

  feedback->trim_thrusts.resize(drone_.numRotors());
  feedback->delta_thrusts.resize(drone_.numRotors());
  feedback->trim_deflections.resize(drone_.numControlSurfaces());
  feedback->delta_deflections.resize(drone_.numControlSurfaces());

  feedback->trim_u = trim.u();
  feedback->trim_alpha = trim.alpha();

  for (size_t i = 0; i < x_rotors_.count(); ++i)
  {
    const auto& rotor = x_rotors_.rotor(i);
    feedback->trim_thrusts.at(rotor.channel) = eom_.trimInput()(i);
    feedback->delta_thrusts.at(rotor.channel) = du(i);
  }

  for (size_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    const auto u_idx = x_rotors_.count() + i;
    feedback->trim_deflections.at(i) = eom_.trimInput()(u_idx);
    feedback->delta_deflections.at(i) = du(u_idx);
  }

  feedback_pub_->publish(move(feedback));
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
  lqd_.input_weight.head(x_rotors_.count()).fill(thrust_weight);
}

void ControllerNode::updateThrustRateWeightLog10()
{
  const auto thrust_rate_weight = exp10(params_.thrust_rate_weight_log10);
  lqd_.input_rate_weight.head(x_rotors_.count()).fill(thrust_rate_weight);
}

void ControllerNode::updateDeflectionWeightLog10()
{
  const auto deflection_weight = exp10(params_.deflection_weight_log10);
  lqd_.input_weight.tail(drone_.numControlSurfaces()).fill(deflection_weight);
}

void ControllerNode::updateDeflectionRateWeightLog10()
{
  const auto deflection_rate_weight = exp10(params_.deflection_rate_weight_log10);
  lqd_.input_rate_weight.tail(drone_.numControlSurfaces()).fill(deflection_rate_weight);
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
  if (is_initialized_)
    updateForwardSpeedWeight();
  return true;
}

bool ControllerNode::alphaWeightCb(const long& p)
{
  params_.alpha_weight = p;
  if (is_initialized_)
    updateAlphaWeight();
  return true;
}

bool ControllerNode::betaWeightCb(const long& p)
{
  params_.beta_weight = p;
  if (is_initialized_)
    updateBetaWeight();
  return true;
}

bool ControllerNode::attitudeWeightCb(const long& p)
{
  params_.attitude_weight = p;
  if (is_initialized_)
    updateAttitudeWeight();
  return true;
}

bool ControllerNode::angularVelicityWeightCb(const long& p)
{
  params_.angular_velocity_weight = p;
  if (is_initialized_)
    updateAngularVelicityWeight();
  return true;
}

bool ControllerNode::thrustWeightLog10Cb(const long& p)
{
  params_.thrust_weight_log10 = p;
  if (is_initialized_)
    updateThrustWeightLog10();
  return true;
}

bool ControllerNode::thrustRateWeightLog10Cb(const long& p)
{
  params_.thrust_rate_weight_log10 = p;
  if (is_initialized_)
    updateThrustRateWeightLog10();
  return true;
}

bool ControllerNode::deflectionWeightLog10Cb(const long& p)
{
  params_.deflection_weight_log10 = p;
  if (is_initialized_)
    updateDeflectionWeightLog10();
  return true;
}

bool ControllerNode::deflectionRateWeightLog10Cb(const long& p)
{
  params_.deflection_rate_weight_log10 = p;
  if (is_initialized_)
    updateDeflectionRateWeightLog10();
  return true;
}

void ControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;

  if (tree_received_)
  {
    if (!initialize())
    {
      TOBAS_FATAL("Error occured while initializing controller.");
      return;
    }
  }

  drone_received_ = true;
}

void ControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  if (drone_received_)
  {
    if (!initialize())
    {
      TOBAS_FATAL("Error occured while initializing controller.");
      return;
    }
  }

  tree_received_ = true;
}

void ControllerNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;

  if (!arming->data)
  {
    cmd_nwu_ = nullptr;
    lqd_.last_input.setZero();
  }
}

void ControllerNode::airPressureCb(const tobas_msgs::msg::FluidPressureWithVarianceStamped::ConstSharedPtr& pressure)
{
  air_pressure_ = pressure;
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

  // アームされていなければスキップ
  if (!arming_->data)
    return;

  // コマンドが来ていなければスキップ
  if (cmd_nwu_ == nullptr)
    return;

  // NWU -> NED
  tobas::odometryNwuToNed(*odom_nwu_, odom_ned_);
  tobas::speedRollDeltaPitchNwuToNed(*cmd_nwu_, cmd_ned_);

  // 現在の速度を使って状態方程式を更新
  const auto rho = tobas_std::pressureToDensity(air_pressure_->pressure.pressure);
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

  const VectorXd thrusts = u.block(0, 0, x_rotors_.count(), 1);
  const VectorXd deflections = u.block(x_rotors_.count(), 0, drone_.numControlSurfaces(), 1);

  // Publish
  publishThrusts(thrusts);
  publishDeflections(deflections);
  publishFeedback(du);
}

void ControllerNode::commandCb(const tobas_msgs::msg::SpeedRollDeltaPitch::ConstSharedPtr& cmd_nwu)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!arming_->data)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the rotors are disarmed.");
    return;
  }

  // TODO: コマンドレベルの処理

  cmd_nwu_ = cmd_nwu;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
