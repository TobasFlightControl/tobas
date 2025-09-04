#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_eigen_tools/kinematics.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_node/node.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_command_msgs_adapter/accel_yaw.hpp>
#include <tobas_command_msgs_adapter/angle_throttle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>
#include <tobas_command_msgs_adapter/rate_throttle.hpp>
#include <tobas_debug_msgs_adapter/multicopter_controller_feedback.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

#include "tobas_planar_multi_controller/mixer_qp.hpp"
#include "tobas_planar_multi_controller/translational_eom.hpp"

namespace tobas
{
namespace planar_multicopter
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
  TreeJointStateConverter js_converter_;

  // Static parameters
  bool do_dist_comp_trans_;
  bool do_dist_comp_rot_;
  bool standard_second_order_form_tuning_;

  // Controller
  PositionPID pos_pid_;
  TranslationalEoM trans_eom_;
  QpMixer mixer_;
  double atti_wn_, head_wn_;      // [rad/s]
  double atti_zeta_, head_zeta_;  // [-]
  kdl::Vector angle_gain_, rate_gain_;  // 回転のPIDを2段階に分けたときの姿勢と角速度に対応したゲイン
  kdl::Vector rot_ki_, rot_ei_;
  double throttle_gain_thresh_;  // [-]

  // Values depending on drone configuration
  double max_thrust_sum_;  // [N]

  // State
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool js_received_ = false;
  bool topics_received_ = false;
  CommandLevelHandler cmd_level_handler_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  // Command
  tobas_command_msgs::PosVelYaw::SharedPtr pos_cmd_;  // 位置制御の目標値 (世界座標系)
  tobas_command_msgs::AccelYaw::SharedPtr acc_cmd_;   // 加速度制御の目標値 (世界座標系)
  std::shared_ptr<kdl::Euler> tar_angle_;             // 目標オイラー角 (世界座標系)
  std::shared_ptr<kdl::Vector> tar_gyro_;             // 目標角速度 (機体座標系P)
  kdl::Vector tar_dgyro_;                             // 目標角加速度
  double tar_thrust_;                                 // 目標推力

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::MulticopterControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_livelinesses_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::PosVelYaw> pos_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::AccelYaw> acc_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::AngleThrottle> angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::RateThrottle> rate_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool updateInternalDataStructures();
  void resetCommands();
  void resetIntegralErrors();
  bool isCommandAccepted(const tobas_command_msgs::msg::CommandLevel& level);
  std::pair<kdl::Vector, kdl::Vector> computeRotGain() const;
  static kdl::Vector computeEulerError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy);

  // Gain (Second-order form) parameter callbacks
  bool horizontalNaturalFreqCb(const double& p);
  bool horizontalDampingRatioCb(const double& p);
  bool horizontalIGainCb(const double& p);
  bool verticalNaturalFreqCb(const double& p);
  bool verticalDampingRatioCb(const double& p);
  bool verticalIGainCb(const double& p);
  bool attitudeNaturalFreqCb(const double& p);
  bool attitudeDampingRatioCb(const double& p);
  bool attitudeIGainCb(const double& p);
  bool headingNaturalFreqCb(const double& p);
  bool headingDampingRatioCb(const double& p);
  bool headingIGainCb(const double& p);

  // Gain (PID form) parameter callbacks
  bool xPGainCb(const double& p);
  bool xIGainCb(const double& p);
  bool xDGainCb(const double& p);
  bool yPGainCb(const double& p);
  bool yIGainCb(const double& p);
  bool yDGainCb(const double& p);
  bool zPGainCb(const double& p);
  bool zIGainCb(const double& p);
  bool zDGainCb(const double& p);
  bool rollAngleGainCb(const double& p);
  bool rollRateGainCb(const double& p);
  bool rollIntegralGainCb(const double& p);
  bool pitchAngleGainCb(const double& p);
  bool pitchRateGainCb(const double& p);
  bool pitchIntegralGainCb(const double& p);
  bool yawAngleGainCb(const double& p);
  bool yawRateGainCb(const double& p);
  bool yawIntegralGainCb(const double& p);

  // Limit parameter callbacks
  bool maxHorizontalAccelCb(const double& p);
  bool maxVerticalAccelCb(const double& p);
  bool maxAttitudeCb(const double& p);

  // Other parameter callbacks
  bool throttleGainThresholdCb(const double& p);

  // Topic callbacks
  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_livelinesses);
  void positionCommandCb(const tobas_command_msgs::PosVelYaw::ConstSharedPtr& pos_cmd);
  void accelCommandCb(const tobas_command_msgs::AccelYaw::ConstSharedPtr& acc_cmd);
  void angleCommandCb(const tobas_command_msgs::AngleThrottle::ConstSharedPtr& angle_cmd);
  void rateCommandCb(const tobas_command_msgs::RateThrottle::ConstSharedPtr& rate_cmd);

  // Timer callbacks
  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(node::kController, options)
  , mass_holder_(tree_)
  , js_converter_(tree_)
  , trans_eom_(tree_)
  , mixer_(drone_, tree_)
{
  // Get static parameters
  do_dist_comp_trans_ = getBoolParam("do_disturbance_compensation_translation");
  do_dist_comp_rot_ = getBoolParam("do_disturbance_compensation_rotation");
  standard_second_order_form_tuning_ = getBoolParam("standard_second_order_form_tuning");

  // Iゲインは1~2秒で補正が感じられるくらいに設定するのが良いらしい (GPT o1)
  const long default_horizontal_i_gain = do_dist_comp_trans_ ? 0 : 10;
  const long default_vertical_i_gain = do_dist_comp_trans_ ? 0 : 10;
  const long default_attitude_i_gain = do_dist_comp_rot_ ? 0 : 10;
  const long default_heading_i_gain = do_dist_comp_rot_ ? 0 : 10;

  // Register dynamic parameters
  if (standard_second_order_form_tuning_) {
    addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFreqCb, this, 0.2, 5, 1, 25, " rad/s");
    addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFreqCb, this, 0.2, 5, 1, 25, " rad/s");
    addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFreqCb, this, 1., 10, 1, 50, " rad/s");
    addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFreqCb, this, 0.5, 10, 1, 50, " rad/s");
    addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 0.1, 10, 1, 30);
    addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 0.1, 10, 1, 30);
    addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 0.1, 10, 1, 30);
    addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 0.1, 10, 1, 30);
    addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0.01, default_horizontal_i_gain, 0, 30);
    addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0.01, default_vertical_i_gain, 0, 30);
    addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0.1, default_attitude_i_gain, 0, 30);
    addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0.01, default_heading_i_gain, 0, 30);
  }
  else {
    addDynamicDoubleParam("x/p_gain", &self::xPGainCb, this, 0.2, 5, 1, 30);
    addDynamicDoubleParam("x/i_gain", &self::xIGainCb, this, 0.01, default_horizontal_i_gain, 0, 30);
    addDynamicDoubleParam("x/d_gain", &self::xDGainCb, this, 0.2, 10, 1, 30);
    addDynamicDoubleParam("y/p_gain", &self::yPGainCb, this, 0.2, 5, 1, 30);
    addDynamicDoubleParam("y/i_gain", &self::yIGainCb, this, 0.01, default_horizontal_i_gain, 0, 30);
    addDynamicDoubleParam("y/d_gain", &self::yDGainCb, this, 0.2, 10, 1, 30);
    addDynamicDoubleParam("z/p_gain", &self::zPGainCb, this, 0.2, 5, 1, 30);
    addDynamicDoubleParam("z/i_gain", &self::zIGainCb, this, 0.01, default_vertical_i_gain, 0, 30);
    addDynamicDoubleParam("z/d_gain", &self::zDGainCb, this, 0.2, 10, 1, 30);
    addDynamicDoubleParam("roll/angle_gain", &self::rollAngleGainCb, this, 0.5, 10, 1, 50);
    addDynamicDoubleParam("roll/rate_gain", &self::rollRateGainCb, this, 2., 10, 1, 50);
    addDynamicDoubleParam("roll/integral_gain", &self::rollIntegralGainCb, this, 0.1, default_attitude_i_gain, 0, 30);
    addDynamicDoubleParam("pitch/angle_gain", &self::pitchAngleGainCb, this, 0.5, 10, 1, 50);
    addDynamicDoubleParam("pitch/rate_gain", &self::pitchRateGainCb, this, 2., 10, 1, 50);
    addDynamicDoubleParam("pitch/integral_gain", &self::pitchIntegralGainCb, this, 0.1, default_attitude_i_gain, 0, 30);
    addDynamicDoubleParam("yaw/angle_gain", &self::yawAngleGainCb, this, 0.25, 10, 1, 50);
    addDynamicDoubleParam("yaw/rate_gain", &self::yawRateGainCb, this, 1., 10, 1, 50);
    addDynamicDoubleParam("yaw/integral_gain", &self::yawIntegralGainCb, this, 0.01, default_heading_i_gain, 0, 30);
  }
  addDynamicDoubleParam("max_horizontal_accel", &self::maxHorizontalAccelCb, this, 0.5, 16, 2, 40, " m/s^2");
  addDynamicDoubleParam("max_vertical_accel", &self::maxVerticalAccelCb, this, 0.5, 8, 2, 20, " m/s^2");
  addDynamicDoubleParam("max_attitude", &self::maxAttitudeCb, this, 1., 60, 0, 90, " deg");
  addDynamicDoubleParam("throttle_gain_threshold", &self::throttleGainThresholdCb, this, 1., 50, 0, 100, " %");

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(kRotorThrustsCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MulticopterControllerFeedback>(kMRCtrlFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(kKdlTreeTopic, &self::treeCb, this, true, true);
  odom_sub_ = createSubscriber(kOdometryTopic, &self::odomCb, this);
  if (do_dist_comp_trans_ || do_dist_comp_rot_) {
    dist_force_sub_ = createSubscriber(kDisturbanceForceTopic, &self::disturbanceForceCb, this);
  }
  landed_sub_ = createSubscriber(kLandedTopic, &self::landedCb, this);
  arming_sub_ = createSubscriber(kArmingTopic, &self::armingCb, this);
  rotor_livelinesses_sub_ = createSubscriber(kRotorLivelinessesTopic, &self::rotorLivelinessCb, this);
  pos_cmd_sub_ = createSubscriber(kPosVelYawCmdTopic, &self::positionCommandCb, this);
  acc_cmd_sub_ = createSubscriber(kAccelYawCmdTopic, &self::accelCommandCb, this);
  angle_cmd_sub_ = createSubscriber(kAngleThrottleCmdTopic, &self::angleCommandCb, this);
  rate_cmd_sub_ = createSubscriber(kRateThrottleCmdTopic, &self::rateCommandCb, this);

  // Register timers
  check_topics_timer_ = createTimer(kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::updateInternalDataStructures()
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

  // Update the maximum total thrust
  max_thrust_sum_ = 0.;
  for (const auto& [link_name, _] : drone_.prop->rotors) {
    const auto thrust_at_full_throt = drone_.prop->thrustFromThrottle(link_name, kMaxThrot);
    max_thrust_sum_ += thrust_at_full_throt;
  }

  return true;
}

bool ControllerNode::isCommandAccepted(const tobas_command_msgs::msg::CommandLevel& level)
{
  if (!topics_received_) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because some topics are not received yet.");
    return false;
  }

  if (!arming_->data) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because the rotors are disarmed.");
    return false;
  }

  if (!cmd_level_handler_.update(level.data, get_clock()->now())) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return false;
  }

  return true;
}

std::pair<kdl::Vector, kdl::Vector> ControllerNode::computeRotGain() const
{
  // 目標推力が重量の割合で定められた閾値未満のときは，推力が小さいほど姿勢制御の自然周波数が小さくなるように調整する．
  // これで低速域でのジャイロに対する可変ピッチの感度が一定になる (memo: 3-33)

  double thrust_coef;
  if (throttle_gain_thresh_ > 0.) {
    const auto thrust_thresh = mass_holder_.getMass() * tobas_std::kGravity * throttle_gain_thresh_;
    if (tar_thrust_ > thrust_thresh) {
      thrust_coef = 1.;
    }
    else {
      thrust_coef = tar_thrust_ / thrust_thresh;
    }
  }
  else {
    thrust_coef = 1.;
  }

  kdl::Vector angle_gain, rate_gain;
  if (standard_second_order_form_tuning_) {
    const auto atti_wn = atti_wn_ * thrust_coef;
    const auto head_wn = head_wn_ * thrust_coef;
    const auto atti_angle_gain = atti_wn / atti_zeta_ / 2;
    const auto head_angle_gain = head_wn / head_zeta_ / 2;
    const auto atti_rate_gain = atti_wn * atti_zeta_ * 2;
    const auto head_rate_gain = head_wn * head_zeta_ * 2;
    angle_gain = { atti_angle_gain, atti_angle_gain, head_angle_gain };
    rate_gain = { atti_rate_gain, atti_rate_gain, head_rate_gain };
  }
  else {
    angle_gain = angle_gain_ * thrust_coef;
    rate_gain = rate_gain_ * thrust_coef;
  }

  return { angle_gain, rate_gain };
}

kdl::Vector ControllerNode::computeEulerError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy)
{
  // 2つのオイラー角を結ぶ直線は回転における最短距離ではないことに注意
  const auto roll_err = algo::wrapPi(tar_rpy.roll - cur_rpy.roll);
  const auto pitch_err = algo::wrapPi(tar_rpy.pitch - cur_rpy.pitch);
  const auto yaw_err = algo::wrapPi(tar_rpy.yaw - cur_rpy.yaw);
  return { roll_err, pitch_err, yaw_err };
}

void ControllerNode::resetCommands()
{
  pos_cmd_.reset();
  acc_cmd_.reset();
  tar_angle_.reset();
  tar_gyro_.reset();
}

void ControllerNode::resetIntegralErrors()
{
  pos_pid_.resetIntegralError();
  rot_ei_.setZero();
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

bool ControllerNode::attitudeNaturalFreqCb(const double& p)
{
  atti_wn_ = p;
  return true;
}

bool ControllerNode::attitudeDampingRatioCb(const double& p)
{
  atti_zeta_ = p;
  return true;
}

bool ControllerNode::attitudeIGainCb(const double& p)
{
  rot_ki_.x(p);
  rot_ki_.y(p);
  return true;
}

bool ControllerNode::headingNaturalFreqCb(const double& p)
{
  head_wn_ = p;
  return true;
}

bool ControllerNode::headingDampingRatioCb(const double& p)
{
  head_zeta_ = p;
  return true;
}

bool ControllerNode::headingIGainCb(const double& p)
{
  rot_ki_.z(p);
  return true;
}

bool ControllerNode::xPGainCb(const double& p)
{
  return pos_pid_.setProportionalGain(0, p);
}

bool ControllerNode::xIGainCb(const double& p)
{
  return pos_pid_.setIntegralGain(0, p);
}

bool ControllerNode::xDGainCb(const double& p)
{
  return pos_pid_.setDerivativeGain(0, p);
}

bool ControllerNode::yPGainCb(const double& p)
{
  return pos_pid_.setProportionalGain(1, p);
}

bool ControllerNode::yIGainCb(const double& p)
{
  return pos_pid_.setIntegralGain(1, p);
}

bool ControllerNode::yDGainCb(const double& p)
{
  return pos_pid_.setDerivativeGain(1, p);
}

bool ControllerNode::zPGainCb(const double& p)
{
  return pos_pid_.setProportionalGain(2, p);
}

bool ControllerNode::zIGainCb(const double& p)
{
  return pos_pid_.setIntegralGain(2, p);
}

bool ControllerNode::zDGainCb(const double& p)
{
  return pos_pid_.setDerivativeGain(2, p);
}

bool ControllerNode::rollAngleGainCb(const double& p)
{
  angle_gain_.x(p);
  return true;
}

bool ControllerNode::rollRateGainCb(const double& p)
{
  rate_gain_.x(p);
  return true;
}

bool ControllerNode::rollIntegralGainCb(const double& p)
{
  rot_ki_.x(p);
  return true;
}

bool ControllerNode::pitchAngleGainCb(const double& p)
{
  angle_gain_.y(p);
  return true;
}

bool ControllerNode::pitchRateGainCb(const double& p)
{
  rate_gain_.y(p);
  return true;
}

bool ControllerNode::pitchIntegralGainCb(const double& p)
{
  rot_ki_.y(p);
  return true;
}

bool ControllerNode::yawAngleGainCb(const double& p)
{
  angle_gain_.z(p);
  return true;
}

bool ControllerNode::yawRateGainCb(const double& p)
{
  rate_gain_.z(p);
  return true;
}

bool ControllerNode::yawIntegralGainCb(const double& p)
{
  rot_ki_.z(p);
  return true;
}

bool ControllerNode::maxHorizontalAccelCb(const double& p)
{
  return pos_pid_.setMaximumAccel(0, p) && pos_pid_.setMaximumAccel(1, p);
}

bool ControllerNode::maxVerticalAccelCb(const double& p)
{
  return pos_pid_.setMaximumAccel(2, p);
}

bool ControllerNode::maxAttitudeCb(const double& p)
{
  return trans_eom_.setMaxAttitude(tobas_std::deg2rad(p));
}

bool ControllerNode::throttleGainThresholdCb(const double& p)
{
  throttle_gain_thresh_ = p / 100.;
  return true;
}

void ControllerNode::droneCb(const Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;

  if (drone->hasServoJoint()) {
    js_sub_ = createSubscriber(kJointStatesTopic, &self::jointStateCb, this);
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

void ControllerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (!odom_) {
    odom_ = odom;
    return;
  }

  // 経過時間を計算してオドメトリを更新
  const auto dt = (odom->header.stamp - odom_->header.stamp).seconds();
  odom_ = odom;

  // フィードバックメッセージを作成
  auto feedback = std::make_unique<tobas_debug_msgs::MulticopterControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;

  // 位置制御器
  if (pos_cmd_) {
    if (!acc_cmd_) {
      acc_cmd_ = std::make_shared<tobas_command_msgs::AccelYaw>();
    }

    // 世界座標系から見た現在の位置速度
    const auto& cur_pos_W = odom->frame.p;
    const auto cur_vel_W = odom->frame.M * odom->twist.vel;

    // 目標加速度を計算
    // 接地している場合はI制御は行わない
    if (landed_->data) {
      acc_cmd_->accel = pos_pid_.updatePD(cur_pos_W, cur_vel_W, pos_cmd_->pos, pos_cmd_->vel);
    }
    else {
      acc_cmd_->accel = pos_pid_.updatePID(cur_pos_W, cur_vel_W, pos_cmd_->pos, pos_cmd_->vel, dt);
    }

    // ヨー角はそのまま流す
    acc_cmd_->yaw = pos_cmd_->yaw;

    // フィードバックメッセージを埋める
    feedback->target_position = pos_cmd_->pos;
    feedback->target_velocity = odom->frame.M.inverse(pos_cmd_->vel);
    feedback->position_integral_error = pos_pid_.getIntegralError();
  }

  // 加速度制御器
  if (acc_cmd_) {
    if (!tar_angle_) {
      tar_angle_ = std::make_shared<kdl::Euler>();
    }

    // 推力和と目標姿勢を計算
    const auto& dist_force_W = do_dist_comp_trans_ ? dist_force_->wrench.force : kdl::Vector::Zero();
    trans_eom_.update(odom->frame.M, acc_cmd_->accel, dist_force_W, tar_thrust_, tar_angle_->roll, tar_angle_->pitch);

    // ヨー角はそのまま流す
    tar_angle_->yaw = acc_cmd_->yaw;

    // フィードバックメッセージを埋める
    feedback->target_accel = odom->frame.M.inverse(acc_cmd_->accel);
  }

  // 回転のゲインを決定
  const auto [angle_gain, rate_gain] = computeRotGain();

  // 姿勢制御器
  if (tar_angle_) {
    if (!tar_gyro_) {
      tar_gyro_ = std::make_shared<kdl::Vector>();
    }

    // 現在のオイラー角を計算
    const kdl::Euler cur_rpy(odom->frame.M);

    // 誤差を計算
    const auto ep = computeEulerError(cur_rpy, *tar_angle_);

    // 浮遊していれば積分誤差を蓄積
    if (!landed_->data) {
      for (int i = 0; i < 3; ++i) {
        if (rot_ki_(i) > 0.) {
          rot_ei_(i) += ep(i) * dt;
        }
        else {
          rot_ei_(i) = 0.;
        }
      }
    }

    // 目標オイラー角速度を計算 (接地している場合はI制御は行わない)
    const auto tar_drpy = angle_gain.hadamard(ep) + rot_ki_.hadamard(rot_ei_);

    // オイラー角速度をジャイロに変換
    *tar_gyro_ = eigen::angvelFromEulerrateLocal(tar_drpy.data, cur_rpy.roll, cur_rpy.pitch);

    // フィードバックメッセージを埋める
    feedback->target_angle = *tar_angle_;
    feedback->angle_integral_error = rot_ei_;
  }

  if (tar_gyro_) {
    // 角速度制御器
    {
      // 目標角加速度を計算
      tar_dgyro_ = rate_gain.hadamard(*tar_gyro_ - odom->twist.rot);

      // フィードバックメッセージを埋める
      feedback->target_gyro = *tar_gyro_;
    }

    // ミキサー
    {
      const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
      if (!mixer_.solve(js_converter_.getPosition(), odom->twist.rot, tar_dgyro_, tar_thrust_, dist_torque_B)) {
        TOBAS_FATAL("Failed to solve mixing equation.");
        return;
      }

      // フィードバックメッセージを埋める
      feedback->target_dgyro = tar_dgyro_;
    }

    // 目標推力を発行
    auto thrusts_msg = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    thrusts_msg->header.stamp = odom->header.stamp;
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      thrusts_msg->thrusts.emplace_back();
      thrusts_msg->thrusts.back().link_name = rotor_it.first;
      thrusts_msg->thrusts.back().thrust = mixer_.getThrust(idx);
    }
    tar_thrusts_pub_->publish(std::move(thrusts_msg));

    // フィードバックメッセージを発行
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
  if (landed->data) {
    resetIntegralErrors();
  }

  landed_ = landed;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  if (arming_ && arming_->data && !arming->data) {
    resetCommands();
    resetIntegralErrors();
    TOBAS_INFO("Controller is reset.");
  }

  arming_ = arming;
}

void ControllerNode::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_livelinesses)
{
  if (!mixer_.isInitialized()) {
    return;
  }

  for (const auto& data : rotor_livelinesses->data) {
    if (!mixer_.setRotorLiveliness(data.link_name, data.alive)) {
      TOBAS_ERROR("Failed to set the liveliness of rotor \"", data.link_name, "\".");
    }
  }
}

void ControllerNode::positionCommandCb(const tobas_command_msgs::PosVelYaw::ConstSharedPtr& pos_cmd)
{
  if (!isCommandAccepted(pos_cmd->level)) {
    return;
  }

  // コマンドを更新
  pos_cmd_ = std::make_shared<tobas_command_msgs::PosVelYaw>(*pos_cmd);
}

void ControllerNode::accelCommandCb(const tobas_command_msgs::AccelYaw::ConstSharedPtr& acc_cmd)
{
  if (!isCommandAccepted(acc_cmd->level)) {
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();

  // コマンドを更新
  acc_cmd_ = std::make_shared<tobas_command_msgs::AccelYaw>(*acc_cmd);
}

void ControllerNode::angleCommandCb(const tobas_command_msgs::AngleThrottle::ConstSharedPtr& angle_cmd)
{
  if (!isCommandAccepted(angle_cmd->level)) {
    return;
  }

  // Check command range
  if (fabs(angle_cmd->angle.roll) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (fabs(angle_cmd->angle.pitch) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();
  acc_cmd_.reset();

  // コマンドを更新
  tar_angle_ = std::make_shared<kdl::Euler>(angle_cmd->angle);
  tar_thrust_ = max_thrust_sum_ * std::clamp(angle_cmd->throttle, kMinThrot, kMaxThrot);
}

void ControllerNode::rateCommandCb(const tobas_command_msgs::RateThrottle::ConstSharedPtr& rate_cmd)
{
  if (!isCommandAccepted(rate_cmd->level)) {
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();
  acc_cmd_.reset();
  tar_angle_.reset();

  // コマンドを更新
  tar_gyro_ = std::make_shared<kdl::Vector>(rate_cmd->rate);
  tar_thrust_ = max_thrust_sum_ * std::clamp(rate_cmd->throttle, kMinThrot, kMaxThrot);
}

void ControllerNode::checkTopicsTimerCb()
{
  if (!drone_received_) {
    TOBAS_WARN("Waiting for \"", kDroneTopic, "\".");
    return;
  }

  if (!tree_received_) {
    TOBAS_WARN("Waiting for \"", kKdlTreeTopic, "\".");
    return;
  }

  if (!odom_) {
    TOBAS_WARN("Waiting for \"", kOdometryTopic, "\".");
    return;
  }

  if (dist_force_sub_ && !dist_force_) {
    TOBAS_WARN("Waiting for \"", kDisturbanceForceTopic, "\".");
    return;
  }

  if (js_sub_ && !js_received_) {
    TOBAS_WARN("Waiting for \"", kJointStatesTopic, "\".");
    return;
  }

  if (!landed_) {
    TOBAS_WARN("Waiting for \"", kLandedTopic, "\".");
    return;
  }

  if (!arming_) {
    TOBAS_WARN("Waiting for \"", kArmingTopic, "\".");
    return;
  }

  topics_received_ = true;
  check_topics_timer_->cancel();
}
}  // namespace planar_multicopter
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::planar_multicopter::ControllerNode)
