#include <ranges>

#include <tobas_constants/node.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_node/node.hpp>
#include <tobas_pose_pid/angle_axis_pi.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_ros2_tools/time.hpp>
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

namespace tobas
{
namespace nonplanar_multicopter
{
class ControllerNode : public BaseNode
{
  using self = ControllerNode;
  using super = BaseNode;

  static constexpr long kMaxWeight = 100;

public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  Drone drone_;
  kdl::Tree tree_;

  TreeJointStateConverter js_converter_;

  // Static parameters
  bool do_dist_comp_trans_;
  bool do_dist_comp_rot_;

  // Controllers
  PositionPID pos_pid_;
  AngleAxisPI rot_pi_;
  QpMixer mixer_;
  double atti_wn_, head_wn_;      // [rad/s]
  double atti_zeta_, head_zeta_;  // [-]
  kdl::Vector rate_gain_;

  // Mutable variables
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool js_received_ = false;
  bool topics_received_ = false;
  CommandPriorityHandler cmd_priority_handler_;
  tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr odom_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  // Command
  tobas_command_msgs::PosVelAcc::UniquePtr pos_cmd_;
  tobas_command_msgs::Accel::UniquePtr acc_cmd_;
  tobas_command_msgs::Angle::UniquePtr angle_cmd_;
  tobas_command_msgs::Rate::UniquePtr rate_cmd_;
  std::unique_ptr<kdl::Vector> tar_dgyro_;

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

  bool updateInternalDataStructures();
  bool updateAttitudePDGain();
  bool updateHeadingPDGain();
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& priority);

  bool horizontalNaturalFreqCb(const double& p);
  bool horizontalDampingRatioCb(const double& p);
  bool horizontalIGainCb(const double& p);
  bool horizontalIMaxAccelCb(const double& p);
  bool verticalNaturalFreqCb(const double& p);
  bool verticalDampingRatioCb(const double& p);
  bool verticalIGainCb(const double& p);
  bool verticalIMaxAccelCb(const double& p);
  bool attitudeNaturalFreqCb(const double& p);
  bool attitudeDampingRatioCb(const double& p);
  bool attitudeIGainCb(const double& p);
  bool headingNaturalFreqCb(const double& p);
  bool headingDampingRatioCb(const double& p);
  bool headingIGainCb(const double& p);
  bool mixerLinearWeightCb(const long& p);
  bool mixerAngularWeightCb(const long& p);
  bool mixerThrustWeightLog2Cb(const long& p);
  bool mixerDeltaThrustWeightLog2Cb(const long& p);

  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness);
  void positionCommandCb(const tobas_command_msgs::PosVelAcc::ConstSharedPtr& pos_cmd);
  void accelCommandCb(const tobas_command_msgs::Accel::ConstSharedPtr& acc_cmd);
  void angleCommandCb(const tobas_command_msgs::Angle::ConstSharedPtr& angle_cmd);
  void rateCommandCb(const tobas_command_msgs::Rate::ConstSharedPtr& rate_cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(node::kController, options), js_converter_(tree_), mixer_(drone_, tree_)
{
  // Get static parameters
  do_dist_comp_trans_ = getBoolParam("do_disturbance_compensation_translation");
  do_dist_comp_rot_ = getBoolParam("do_disturbance_compensation_rotation");

  // Register dynamic parameters
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFreqCb, this, 0.2, 5, 1, 30, " rad/s");
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFreqCb, this, 0.2, 5, 1, 30, " rad/s");
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFreqCb, this, 1., 10, 1, 30, " rad/s");
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFreqCb, this, 0.5, 10, 1, 30, " rad/s");
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0.01, 10, 1, 30);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0.01, 10, 1, 30);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0.01, 10, 1, 30);
  addDynamicDoubleParam("horizontal_i_max_accel", &self::horizontalIMaxAccelCb, this, 0.5, 4, 0, 20, " m/s^2");
  addDynamicDoubleParam("vertical_i_max_accel", &self::verticalIMaxAccelCb, this, 0.5, 4, 0, 20, " m/s^2");
  addDynamicIntParam("mixer_linear_weight", &self::mixerLinearWeightCb, this, kMaxWeight / 2, 1, kMaxWeight);
  addDynamicIntParam("mixer_angular_weight", &self::mixerAngularWeightCb, this, kMaxWeight / 2, 1, kMaxWeight);
  addDynamicIntParam("mixer_thrust_weight_log2", &self::mixerThrustWeightLog2Cb, this, -20, -30, 0);
  addDynamicIntParam("mixer_delta_thrust_weight_log2", &self::mixerDeltaThrustWeightLog2Cb, this, -20, -30, 0);

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(topic::kRotorThrustsCmd);
  setpoint_pub_ = createPublisher<tobas_msgs::OdometryStamped>(topic::kTrajSetpoint);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MulticopterControllerFeedback>(topic::kMRCtrlFeedback);

  // Register subscribers
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

  // Register timers
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::updateInternalDataStructures()
{
  if (!js_converter_.updateInternalDataStructures()) {
    return false;
  }
  if (!mixer_.updateInternalDataStructures()) {
    return false;
  }

  return true;
}

bool ControllerNode::updateAttitudePDGain()
{
  // PD制御を2段階に分割したときのゲインを計算 (memo: 3-22)
  const auto angle_gain = atti_wn_ / atti_zeta_ / 2;
  const auto rate_gain = atti_wn_ * atti_zeta_ * 2;

  rate_gain_.x(rate_gain);
  rate_gain_.y(rate_gain);
  return rot_pi_.setProportionalGain(0, angle_gain) && rot_pi_.setProportionalGain(1, angle_gain);
}

bool ControllerNode::updateHeadingPDGain()
{
  // PD制御を2段階に分割したときのゲインを計算 (memo: 3-22)
  const auto angle_gain = head_wn_ / head_zeta_ / 2;
  const auto rate_gain = head_wn_ * head_zeta_ * 2;

  rate_gain_.z(rate_gain);
  return rot_pi_.setProportionalGain(2, angle_gain);
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

bool ControllerNode::horizontalNaturalFreqCb(const double& p)
{
  return pos_pid_.setNaturalFreq(0, p) && pos_pid_.setNaturalFreq(1, p);
}

bool ControllerNode::horizontalDampingRatioCb(const double& p)
{
  return pos_pid_.setDampingRatio(0, p) && pos_pid_.setDampingRatio(1, p);
}

bool ControllerNode::horizontalIGainCb(const double& p)
{
  return pos_pid_.setIntegralGain(0, p) && pos_pid_.setIntegralGain(1, p);
}

bool ControllerNode::horizontalIMaxAccelCb(const double& p)
{
  return pos_pid_.setMaxIntegralAccel(0, p) && pos_pid_.setMaxIntegralAccel(1, p);
}

bool ControllerNode::verticalNaturalFreqCb(const double& p)
{
  return pos_pid_.setNaturalFreq(2, p);
}

bool ControllerNode::verticalDampingRatioCb(const double& p)
{
  return pos_pid_.setDampingRatio(2, p);
}

bool ControllerNode::verticalIGainCb(const double& p)
{
  return pos_pid_.setIntegralGain(2, p);
}

bool ControllerNode::verticalIMaxAccelCb(const double& p)
{
  return pos_pid_.setMaxIntegralAccel(2, p);
}

bool ControllerNode::attitudeNaturalFreqCb(const double& p)
{
  atti_wn_ = p;
  return updateAttitudePDGain();
}

bool ControllerNode::attitudeDampingRatioCb(const double& p)
{
  atti_zeta_ = p;
  return updateAttitudePDGain();
}

bool ControllerNode::attitudeIGainCb(const double& p)
{
  return rot_pi_.setIntegralGain(0, p) && rot_pi_.setIntegralGain(1, p);
}

bool ControllerNode::headingNaturalFreqCb(const double& p)
{
  head_wn_ = p;
  return updateHeadingPDGain();
}

bool ControllerNode::headingDampingRatioCb(const double& p)
{
  head_zeta_ = p;
  return updateHeadingPDGain();
}

bool ControllerNode::headingIGainCb(const double& p)
{
  return rot_pi_.setIntegralGain(2, p);
}

bool ControllerNode::mixerLinearWeightCb(const long& p)
{
  return mixer_.setLinearWeight(static_cast<double>(p) / static_cast<double>(kMaxWeight));
}

bool ControllerNode::mixerAngularWeightCb(const long& p)
{
  return mixer_.setAngularWeight(static_cast<double>(p) / static_cast<double>(kMaxWeight));
}

bool ControllerNode::mixerThrustWeightLog2Cb(const long& p)
{
  return mixer_.setThrustWeight(exp2(p));
}

bool ControllerNode::mixerDeltaThrustWeightLog2Cb(const long& p)
{
  return mixer_.setDeltaThrustWeight(exp2(p));
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
    if (!updateInternalDataStructures()) {
      TOBAS_FATAL("Error occurred while updating internal data structures.");
      return;
    }
  }

  drone_received_ = true;
}

void ControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;

  if (drone_received_) {
    if (!updateInternalDataStructures()) {
      TOBAS_FATAL("Error occurred while updating internal data structures.");
      return;
    }
  }

  tree_received_ = true;
}

void ControllerNode::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  if (!odom_) {
    odom_ = odom;
    return;
  }

  // 経過時間を計算してオドメトリを更新
  const auto& cur_time = odom->header.stamp;
  const auto dt = (cur_time - odom_->header.stamp).seconds();
  odom_ = odom;

  // 設定値メッセージを作成
  auto setpoint = std::make_unique<tobas_msgs::OdometryStamped>();
  setpoint->header.stamp = cur_time;
  setpoint->odom = odom->odom.odom;  // 制御しない値は現在値を入れる

  // フィードバックメッセージを作成
  auto feedback = std::make_unique<tobas_debug_msgs::MulticopterControllerFeedback>();
  feedback->header.stamp = cur_time;

  // エイリアス
  const auto& cur_pos_W = odom->odom.odom.frame.p;
  const auto& cur_rot = odom->odom.odom.frame.M;
  const auto& cur_vel_B = odom->odom.odom.twist.vel;
  const auto& cur_gyro_B = odom->odom.odom.twist.rot;

  // 位置制御器
  if (pos_cmd_) {
    if (!acc_cmd_) {
      acc_cmd_ = std::make_unique<tobas_command_msgs::Accel>();
    }

    // 世界座標系から見た現在の位置速度
    const auto cur_vel_W = cur_rot * cur_vel_B;

    // 目標加速度を計算（接地している場合は誤差の積分を行わない）
    acc_cmd_->accel =
      pos_cmd_->acc + pos_pid_.update(cur_pos_W, cur_vel_W, pos_cmd_->pos, pos_cmd_->vel, landed_->landed ? 0. : dt);

    // フィードバックメッセージを埋める
    setpoint->odom.frame.p = pos_cmd_->pos;
    setpoint->odom.twist.vel = cur_rot.inverse(pos_cmd_->vel);
    feedback->position_integral_error = pos_pid_.getIntegralError();
  }

  // 姿勢制御器
  if (angle_cmd_) {
    if (!rate_cmd_) {
      rate_cmd_ = std::make_unique<tobas_command_msgs::Rate>();
    }

    // 目標角速度を計算（接地している場合は誤差の積分を行わない）
    const auto tar_rot = angle_cmd_->angle.toRotation();
    rate_cmd_->rate = rot_pi_.update(cur_rot, tar_rot, landed_->landed ? 0. : dt);

    // フィードバックメッセージを埋める
    setpoint->odom.frame.M = tar_rot;
    feedback->angle_integral_error = rot_pi_.getIntegralError();
  }

  // 角速度制御器
  if (rate_cmd_) {
    if (!tar_dgyro_) {
      tar_dgyro_ = std::make_unique<kdl::Vector>();
    }

    // 目標角加速度を計算
    *tar_dgyro_ = rate_gain_.hadamard(rate_cmd_->rate - cur_gyro_B);

    // フィードバックメッセージを埋める
    setpoint->odom.twist.rot = rate_cmd_->rate;
  }

  // ミキサー
  if (acc_cmd_ && tar_dgyro_) {
    // 6軸加速度をプロペラの推力に変換
    const auto& dist_force_W = do_dist_comp_trans_ ? dist_force_->wrench.force : kdl::Vector::Zero();
    const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
    if (!mixer_.solve(
          js_converter_.getPosition(), cur_rot, cur_gyro_B, acc_cmd_->accel, *tar_dgyro_, dist_force_W, dist_torque_B)) {
      TOBAS_FATAL("Failed to solve the mixing equation.");
      return;
    }
    const auto& thrusts = mixer_.getThrusts();

    // 目標推力を発行
    auto tar_thrusts = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    tar_thrusts->header.stamp = cur_time;
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      tar_thrusts->thrusts.emplace_back();
      tar_thrusts->thrusts.back().link_name = rotor_it.first;
      tar_thrusts->thrusts.back().thrust = std::max(thrusts(idx), 0.);
    }
    tar_thrusts_pub_->publish(std::move(tar_thrusts));

    // フィードバックメッセージを埋める
    setpoint->odom.accel.linear = cur_rot.inverse(acc_cmd_->accel);
    setpoint->odom.accel.angular = *tar_dgyro_;

    // フィードバックメッセージを発行
    setpoint_pub_->publish(std::move(setpoint));
    feedback_pub_->publish(std::move(feedback));
  }
}

void ControllerNode::disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force)
{
  dist_force_ = dist_force;
}

void ControllerNode::jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js)
{
  // 異なる関節の情報が別々のメッセージで送られてくる場合を想定し，メッセージそのものを保持せずにコールバックでKDLへの変換まで行う．
  if (js_converter_.convert(*js) < 0) {
    TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());
    return;
  }

  js_received_ = true;
}

void ControllerNode::landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed)
{
  landed_ = landed;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  if (!arming_) {
    arming_ = arming;
    return;
  }

  // ディスアームしたら積分誤差とコマンドをリセット
  if (!arming->data && arming_->data) {
    pos_pid_.resetIntegralError();
    rot_pi_.resetIntegralError();

    pos_cmd_.reset();
    acc_cmd_.reset();
    angle_cmd_.reset();
    rate_cmd_.reset();
    tar_dgyro_.reset();

    TOBAS_INFO("The controller has been reset.");
  }

  arming_ = arming;
}

void ControllerNode::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness)
{
  if (!mixer_.isInitialized()) {
    return;
  }

  for (const auto& data : rotor_liveliness->data) {
    if (!mixer_.setRotorLiveliness(data.link_name, data.alive)) {
      TOBAS_ERROR("Failed to set the liveliness of rotor \"", data.link_name, "\".");
    }
  }
}

void ControllerNode::positionCommandCb(const tobas_command_msgs::PosVelAcc::ConstSharedPtr& pos_cmd)
{
  if (!isCommandAccepted(pos_cmd->priority)) {
    return;
  }

  // コマンドを更新
  pos_cmd_ = std::make_unique<tobas_command_msgs::PosVelAcc>(*pos_cmd);
}

void ControllerNode::accelCommandCb(const tobas_command_msgs::Accel::ConstSharedPtr& acc_cmd)
{
  if (!isCommandAccepted(acc_cmd->priority)) {
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();

  // コマンドを更新
  acc_cmd_ = std::make_unique<tobas_command_msgs::Accel>(*acc_cmd);
}

void ControllerNode::angleCommandCb(const tobas_command_msgs::Angle::ConstSharedPtr& angle_cmd)
{
  if (!isCommandAccepted(angle_cmd->priority)) {
    return;
  }

  // コマンドを更新
  angle_cmd_ = std::make_unique<tobas_command_msgs::Angle>(*angle_cmd);
}

void ControllerNode::rateCommandCb(const tobas_command_msgs::Rate::ConstSharedPtr& rate_cmd)
{
  if (!isCommandAccepted(rate_cmd->priority)) {
    return;
  }

  // 外側の制御を止める
  angle_cmd_.reset();

  // コマンドを更新
  rate_cmd_ = std::make_unique<tobas_command_msgs::Rate>(*rate_cmd);
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

  if (!odom_) {
    TOBAS_WARN("Waiting for \"", topic::kOdometry, "\".");
    return;
  }

  if (dist_force_sub_ && !dist_force_) {
    TOBAS_WARN("Waiting for \"", topic::kDisturbanceForce, "\".");
    return;
  }

  if (js_sub_ && !js_received_) {
    TOBAS_WARN("Waiting for \"", topic::kJointStates, "\".");
    return;
  }

  if (!landed_) {
    TOBAS_WARN("Waiting for \"", topic::kLanded, "\".");
    return;
  }

  if (!arming_) {
    TOBAS_WARN("Waiting for \"", topic::kArming, "\".");
    return;
  }

  topics_received_ = true;
  check_topics_timer_->cancel();
}
}  // namespace nonplanar_multicopter
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::nonplanar_multicopter::ControllerNode)
