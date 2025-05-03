#include <ranges>

#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_tools/tr_mixer_pinv.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_pose_pid/angle_axis_pi.hpp>

#include <tobas_std_msgs/msg/bool_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_command_msgs_adapter/pos_vel.hpp>
#include <tobas_command_msgs_adapter/accel.hpp>
#include <tobas_command_msgs_adapter/angle.hpp>
#include <tobas_command_msgs_adapter/rate.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_debug_msgs_adapter/multi_rotor_controller_feedback.hpp>

using namespace std;
using namespace Eigen;

class ControllerNode : public tobas::BaseNode
{
  using self = ControllerNode;
  using super = tobas::BaseNode;

public:
  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  tobas::Drone drone_;
  kdl::Tree tree_;

  tobas::TreeJointStateConverter js_converter_;

  // Static parameters
  bool do_dist_comp_trans_;
  bool do_dist_comp_rot_;

  // Controllers
  tobas::PositionPID pos_pid_;
  tobas::AngleAxisPI rot_pi_;
  tobas::TiltRotorMixer_pinv mixer_;
  double atti_wn_, head_wn_;      // [rad/s]
  double atti_zeta_, head_zeta_;  // [-]
  kdl::Vector gyro_gain_;

  // Mutable variables
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool js_received_ = false;
  bool topics_received_ = false;
  tobas::CommandLevelHandler cmd_level_handler_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  tobas_std_msgs::msg::BoolStamped::ConstSharedPtr landed_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  // Command
  tobas_command_msgs::PosVel::SharedPtr pos_cmd_;
  tobas_command_msgs::Accel::SharedPtr acc_cmd_;
  tobas_command_msgs::Angle::SharedPtr angle_cmd_;
  tobas_command_msgs::Rate::SharedPtr rate_cmd_;
  std::shared_ptr<kdl::Vector> tar_dgyro_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> tar_angles_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::MultiRotorControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_std_msgs::msg::BoolStamped> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::PosVel> pos_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::Accel> acc_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::Angle> angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::Rate> rate_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool updateInternalDataStructures();
  bool updateAttitudePDGain();
  bool updateHeadingPDGain();
  void resetCommands();
  void resetIntegralGains();
  bool isCommandAccepted(const tobas_command_msgs::msg::CommandLevel& level);

  bool horizontalNaturalFrequencyCb(const double& p);
  bool horizontalDampingRatioCb(const double& p);
  bool horizontalIGainCb(const double& p);
  bool verticalNaturalFrequencyCb(const double& p);
  bool verticalDampingRatioCb(const double& p);
  bool verticalIGainCb(const double& p);
  bool attitudeNaturalFrequencyCb(const double& p);
  bool attitudeDampingRatioCb(const double& p);
  bool attitudeIGainCb(const double& p);
  bool headingNaturalFrequencyCb(const double& p);
  bool headingDampingRatioCb(const double& p);
  bool headingIGainCb(const double& p);
  bool maxHorizontalAccelCb(const double& p);
  bool maxVerticalAccelCb(const double& p);
  bool tiltAsixSingularDeclinationLBCb(const long& lb_deg);
  bool tiltAsixSingularDeclinationUBCb(const long& ub_deg);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void landedCb(const tobas_std_msgs::msg::BoolStamped::ConstSharedPtr& landed);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness);
  void positionCommandCb(const tobas_command_msgs::PosVel::ConstSharedPtr& pos_cmd);
  void accelCommandCb(const tobas_command_msgs::Accel::ConstSharedPtr& acc_cmd);
  void angleCommandCb(const tobas_command_msgs::Angle::ConstSharedPtr& angle_cmd);
  void rateCommandCb(const tobas_command_msgs::Rate::ConstSharedPtr& rate_cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::node::kController, options), js_converter_(tree_), mixer_(drone_, tree_)
{
  // Get static parameters
  do_dist_comp_trans_ = getBoolParam("do_disturbance_compensation_translation");
  do_dist_comp_rot_ = getBoolParam("do_disturbance_compensation_rotation");

  // Register dynamic parameters
  // XXX: I制御は姿勢変化が大きいと逆効果なため，デフォルトではオフにする．
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFrequencyCb, this, 5., 1., 50.);
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFrequencyCb, this, 5., 0.1, 25.);
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0., 0., 1.);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0., 0., 1.);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0., 0., 10.);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0., 0., 10.);
  addDynamicDoubleParam("max_horizontal_accel", &self::maxHorizontalAccelCb, this, 8., 0., 20.);
  addDynamicDoubleParam("max_vertical_accel", &self::maxVerticalAccelCb, this, 4., 0., 10.);
  addDynamicIntParam("tilt_axis_singular_declination_lb", &self::tiltAsixSingularDeclinationLBCb, this, 10, 0, 45);
  addDynamicIntParam("tilt_axis_singular_declination_ub", &self::tiltAsixSingularDeclinationUBCb, this, 20, 0, 45);

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(tobas::kRotorThrustsCmdTopic);
  tar_angles_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(tobas::kJointPosCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MultiRotorControllerFeedback>(tobas::kMRCtrlFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKdlTreeTopic, &self::treeCb, this, true, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  dist_force_sub_ = createSubscriber(tobas::kDisturbanceForceTopic, &self::disturbanceForceCb, this);
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  rotor_liveliness_sub_ = createSubscriber(tobas::kRotorLivelinessTopic, &self::rotorLivelinessCb, this);
  pos_cmd_sub_ = createSubscriber(tobas::kPosVelCmdTopic, &self::positionCommandCb, this);
  acc_cmd_sub_ = createSubscriber(tobas::kAccelCmdTopic, &self::accelCommandCb, this);
  angle_cmd_sub_ = createSubscriber(tobas::kAngleCmdTopic, &self::angleCommandCb, this);
  rate_cmd_sub_ = createSubscriber(tobas::kRateCmdTopic, &self::rateCommandCb, this);

  // Register timers
  check_topics_timer_ = createTimer(tobas::kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
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
  const auto kp = atti_wn_ / atti_zeta_ / 2;
  const auto kd = atti_wn_ * atti_zeta_ * 2;

  gyro_gain_.x(kd);
  gyro_gain_.y(kd);
  return rot_pi_.setProportionalGain(0, kp) && rot_pi_.setProportionalGain(1, kp);
}

bool ControllerNode::updateHeadingPDGain()
{
  // PD制御を2段階に分割したときのゲインを計算 (memo: 3-22)
  const auto kp = head_wn_ / head_zeta_ / 2;
  const auto kd = head_wn_ * head_zeta_ * 2;

  gyro_gain_.z(kd);
  return rot_pi_.setProportionalGain(2, kp);
}

bool ControllerNode::isCommandAccepted(const tobas_command_msgs::msg::CommandLevel& level)
{
  if (!topics_received_) {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because some topics are not received yet.");
    return false;
  }

  if (!arming_->data) {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the rotors are disarmed.");
    return false;
  }

  if (!cmd_level_handler_.update(level.data, get_clock()->now())) {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return false;
  }

  return true;
}

void ControllerNode::resetCommands()
{
  pos_cmd_.reset();
  acc_cmd_.reset();
  angle_cmd_.reset();
  rate_cmd_.reset();
  tar_dgyro_.reset();
}

void ControllerNode::resetIntegralGains()
{
  pos_pid_.resetIntegralError();
  rot_pi_.resetIntegralError();
}

bool ControllerNode::horizontalNaturalFrequencyCb(const double& p)
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

bool ControllerNode::verticalNaturalFrequencyCb(const double& p)
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

bool ControllerNode::attitudeNaturalFrequencyCb(const double& p)
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

bool ControllerNode::headingNaturalFrequencyCb(const double& p)
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

bool ControllerNode::maxHorizontalAccelCb(const double& p)
{
  return pos_pid_.setMaximumAccel(0, p) && pos_pid_.setMaximumAccel(1, p);
}

bool ControllerNode::maxVerticalAccelCb(const double& p)
{
  return pos_pid_.setMaximumAccel(2, p);
}

bool ControllerNode::tiltAsixSingularDeclinationLBCb(const long& lb_deg)
{
  return mixer_.setTiltAxisSingularDeclinationLB(tobas_std::deg2rad(lb_deg));
}

bool ControllerNode::tiltAsixSingularDeclinationUBCb(const long& ub_deg)
{
  return mixer_.setTiltAxisSingularDeclinationUB(tobas_std::deg2rad(ub_deg));
}

void ControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;

  if (drone->hasServoJoint()) {
    js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::jointStateCb, this);
  }
  else {
    js_sub_.reset();
  }

  if (tree_received_) {
    if (!updateInternalDataStructures()) {
      TOBAS_FATAL("Error occured while updating internal data structures.");
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
      TOBAS_FATAL("Error occured while updating internal data structures.");
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
  auto feedback = std::make_unique<tobas_debug_msgs::MultiRotorControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;

  // 位置制御器
  if (pos_cmd_) {
    if (!acc_cmd_) {
      acc_cmd_ = std::make_shared<tobas_command_msgs::Accel>();
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

    // フィードバックメッセージを埋める
    feedback->target_position = pos_cmd_->pos;
    feedback->target_velocity = odom->frame.M.inverse(pos_cmd_->vel);
    feedback->position_integral_error = pos_pid_.getIntegralError();
  }

  // 姿勢制御器
  if (angle_cmd_) {
    if (!rate_cmd_) {
      rate_cmd_ = std::make_shared<tobas_command_msgs::Rate>();
    }

    // 目標角速度を計算
    // 接地している場合はI制御は行わない
    if (landed_->data) {
      rate_cmd_->rate = rot_pi_.updateP(odom->frame.M, angle_cmd_->angle.toRotation());
    }
    else {
      rate_cmd_->rate = rot_pi_.updatePI(odom->frame.M, angle_cmd_->angle.toRotation(), dt);
    }

    // フィードバックメッセージを埋める
    feedback->target_angle = angle_cmd_->angle;
    feedback->angle_integral_error = rot_pi_.getIntegralError();
  }

  // 角速度制御器
  if (rate_cmd_) {
    if (!tar_dgyro_) {
      tar_dgyro_ = std::make_shared<kdl::Vector>();
    }

    // 目標角加速度を計算
    *tar_dgyro_ = gyro_gain_.hadamard(rate_cmd_->rate - odom->twist.rot);

    // フィードバックメッセージを埋める
    feedback->target_gyro = rate_cmd_->rate;
  }

  // ミキサー
  if (acc_cmd_ && tar_dgyro_) {
    // ミキシング方程式を解く
    const auto& dist_force_W = do_dist_comp_trans_ ? dist_force_->wrench.force : kdl::Vector::Zero();
    const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
    if (!mixer_.solve(
          js_converter_.getPosition(), odom->frame.M, odom->twist.rot, acc_cmd_->accel, *tar_dgyro_, dist_force_W,
          dist_torque_B)) {
      TOBAS_FATAL("Failed to solve mixing equation.");
      return;
    }

    // 推力を発行
    auto tar_thrusts = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    tar_thrusts->header.stamp = odom->header.stamp;
    for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
      tar_thrusts->thrusts.emplace_back();
      tar_thrusts->thrusts.back().link_name = rotor_it.first;
      tar_thrusts->thrusts.back().thrust = mixer_.getThrust(idx);
    }
    tar_thrusts_pub_->publish(move(tar_thrusts));

    // ティルト角を発行
    auto tar_angles = std::make_unique<tobas_msgs::msg::JointCommandArray>();
    tar_angles->header.stamp = odom->header.stamp;
    for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
      const auto& rotor = rotor_it.second;
      if (rotor->tilt_joint_name.empty()) {
        continue;
      }
      tar_angles->commands.emplace_back();
      tar_angles->commands.back().name = rotor->tilt_joint_name;
      tar_angles->commands.back().data = mixer_.getTiltAngle(idx);
    }
    tar_angles_pub_->publish(move(tar_angles));

    // フィードバックメッセージを埋める
    feedback->target_accel = odom->frame.M.inverse(acc_cmd_->accel);
    feedback->target_dgyro = *tar_dgyro_;

    // フィードバックメッセージを発行
    feedback_pub_->publish(move(feedback));
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

void ControllerNode::landedCb(const tobas_std_msgs::msg::BoolStamped::ConstSharedPtr& landed)
{
  if (landed->data) {
    resetIntegralGains();
  }

  landed_ = landed;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  if (arming_ && arming_->data && !arming->data) {
    resetCommands();
    resetIntegralGains();
    TOBAS_INFO("Controller is reset.");
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

void ControllerNode::positionCommandCb(const tobas_command_msgs::PosVel::ConstSharedPtr& pos_cmd)
{
  if (!isCommandAccepted(pos_cmd->level)) {
    return;
  }

  // コマンドを更新
  pos_cmd_ = std::make_shared<tobas_command_msgs::PosVel>(*pos_cmd);
}

void ControllerNode::accelCommandCb(const tobas_command_msgs::Accel::ConstSharedPtr& acc_cmd)
{
  if (!isCommandAccepted(acc_cmd->level)) {
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();

  // コマンドを更新
  acc_cmd_ = std::make_shared<tobas_command_msgs::Accel>(*acc_cmd);
}

void ControllerNode::angleCommandCb(const tobas_command_msgs::Angle::ConstSharedPtr& angle_cmd)
{
  if (!isCommandAccepted(angle_cmd->level)) {
    return;
  }

  // コマンドを更新
  angle_cmd_ = std::make_shared<tobas_command_msgs::Angle>(*angle_cmd);
}

void ControllerNode::rateCommandCb(const tobas_command_msgs::Rate::ConstSharedPtr& rate_cmd)
{
  if (!isCommandAccepted(rate_cmd->level)) {
    return;
  }

  // 外側の制御を止める
  angle_cmd_.reset();

  // コマンドを更新
  rate_cmd_ = std::make_shared<tobas_command_msgs::Rate>(*rate_cmd);
}

void ControllerNode::checkTopicsTimerCb()
{
  if (!drone_received_) {
    TOBAS_WARN("Waiting for \"", tobas::kDroneTopic, "\".");
    return;
  }

  if (!tree_received_) {
    TOBAS_WARN("Waiting for \"", tobas::kKdlTreeTopic, "\".");
    return;
  }

  if (!odom_) {
    TOBAS_WARN("Waiting for \"", tobas::kOdometryTopic, "\".");
    return;
  }

  if (!dist_force_) {
    TOBAS_WARN("Waiting for \"", tobas::kDisturbanceForceTopic, "\".");
    return;
  }

  if (js_sub_ && !js_received_) {
    TOBAS_WARN("Waiting for \"", tobas::kJointStatesTopic, "\".");
    return;
  }

  if (!landed_) {
    TOBAS_WARN("Waiting for \"", tobas::kLandedTopic, "\".");
    return;
  }

  if (!arming_) {
    TOBAS_WARN("Waiting for \"", tobas::kArmingTopic, "\".");
    return;
  }

  topics_received_ = true;
  check_topics_timer_->cancel();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
