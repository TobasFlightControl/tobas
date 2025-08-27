#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_tools/mr_accel_attitude_converter.hpp>
#include <tobas_drone_tools/mr_mixer_qp.hpp>
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
#include <tobas_debug_msgs_adapter/multi_rotor_controller_feedback.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_std_msgs/msg/bool_stamped.hpp>

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
  tobas::TreeJointStateConverter js_converter_;

  // Static parameters
  bool do_dist_comp_trans_;
  bool do_dist_comp_rot_;

  // Controller
  tobas::PositionPID pos_pid_;
  tobas::AccelAttitudeConverter acc_atti_conv_;
  tobas::MultiRotorMixer_QP mixer_;
  double atti_wn_, head_wn_;      // [rad/s]
  double atti_zeta_, head_zeta_;  // [-]
  kdl::Vector rot_ki_;
  kdl::Vector rot_ei_;
  double throttle_gain_thresh_;  // [-]

  // Values depending on drone configuration
  double max_thrust_sum_;  // [N]

  // State
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
  tobas_command_msgs::PosVelYaw::SharedPtr pos_cmd_;  // 位置制御の目標値 (世界座標系)
  tobas_command_msgs::AccelYaw::SharedPtr acc_cmd_;   // 加速度制御の目標値 (世界座標系)
  std::shared_ptr<kdl::Euler> tar_angle_;             // 目標オイラー角 (世界座標系)
  std::shared_ptr<kdl::Vector> tar_gyro_;             // 目標ジャイロ (機体座標系P)
  double tar_thrust_;                                 // 目標推力

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::MultiRotorControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_std_msgs::msg::BoolStamped> landed_sub_;
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
  kdl::Vector computeEulerError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy) const;

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
  bool maxAttitudeCb(const long& p);
  bool throttleGainThresholdCb(const long& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void landedCb(const tobas_std_msgs::msg::BoolStamped::ConstSharedPtr& landed);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_livelinesses);
  void positionCommandCb(const tobas_command_msgs::PosVelYaw::ConstSharedPtr& pos_cmd);
  void accelCommandCb(const tobas_command_msgs::AccelYaw::ConstSharedPtr& acc_cmd);
  void angleCommandCb(const tobas_command_msgs::AngleThrottle::ConstSharedPtr& angle_cmd);
  void rateCommandCb(const tobas_command_msgs::RateThrottle::ConstSharedPtr& rate_cmd);

  void checkTopicsTimerCb();
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::node::kController, options)
  , mass_holder_(tree_)
  , js_converter_(tree_)
  , acc_atti_conv_(tree_)
  , mixer_(drone_, tree_)
{
  // Get static parameters
  do_dist_comp_trans_ = getBoolParam("do_disturbance_compensation_translation");
  do_dist_comp_rot_ = getBoolParam("do_disturbance_compensation_rotation");

  // Iゲインは1~2秒で補正が感じられるくらいに設定するのが良いらしい (GPT o1)
  const long default_horizontal_i_gain = do_dist_comp_trans_ ? 0 : 10;
  const long default_vertical_i_gain = do_dist_comp_trans_ ? 0 : 10;
  const long default_attitude_i_gain = do_dist_comp_rot_ ? 0 : 10;
  const long default_heading_i_gain = do_dist_comp_rot_ ? 0 : 10;

  // Register dynamic parameters
  addDynamicDoubleParam(
    "horizontal_natural_frequency", &self::horizontalNaturalFrequencyCb, this, 0.2, 5, 1, 25, " rad/s");
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFrequencyCb, this, 0.2, 5, 1, 25, " rad/s");
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFrequencyCb, this, 1., 10, 1, 50, " rad/s");
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFrequencyCb, this, 1., 5, 1, 25, " rad/s");
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 0.1, 10, 1, 30);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0.01, default_horizontal_i_gain, 0, 30);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0.01, default_vertical_i_gain, 0, 30);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0.1, default_attitude_i_gain, 0, 30);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0.01, default_heading_i_gain, 0, 30);
  addDynamicDoubleParam("max_horizontal_accel", &self::maxHorizontalAccelCb, this, 0.5, 16, 2, 40, " m/s^2");
  addDynamicDoubleParam("max_vertical_accel", &self::maxVerticalAccelCb, this, 0.5, 8, 2, 20, " m/s^2");
  addDynamicIntParam("max_attitude", &self::maxAttitudeCb, this, 60, 0, 90, " deg");
  addDynamicIntParam("throttle_gain_threshold", &self::throttleGainThresholdCb, this, 50, 0, 100, " %");

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(tobas::kRotorThrustsCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MultiRotorControllerFeedback>(tobas::kMRCtrlFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKdlTreeTopic, &self::treeCb, this, true, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  if (do_dist_comp_trans_ || do_dist_comp_rot_) {
    dist_force_sub_ = createSubscriber(tobas::kDisturbanceForceTopic, &self::disturbanceForceCb, this);
  }
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  rotor_livelinesses_sub_ = createSubscriber(tobas::kRotorLivelinessesTopic, &self::rotorLivelinessCb, this);
  pos_cmd_sub_ = createSubscriber(tobas::kPosVelYawCmdTopic, &self::positionCommandCb, this);
  acc_cmd_sub_ = createSubscriber(tobas::kAccelYawCmdTopic, &self::accelCommandCb, this);
  angle_cmd_sub_ = createSubscriber(tobas::kAngleThrottleCmdTopic, &self::angleCommandCb, this);
  rate_cmd_sub_ = createSubscriber(tobas::kRateThrottleCmdTopic, &self::rateCommandCb, this);

  // Register timers
  check_topics_timer_ = createTimer(tobas::kCheckTopicsPeriod, &self::checkTopicsTimerCb, this);
}

bool ControllerNode::updateInternalDataStructures()
{
  if (!mass_holder_.updateInternalDataStructures()) {
    return false;
  }
  if (!js_converter_.updateInternalDataStructures()) {
    return false;
  }
  if (!acc_atti_conv_.updateInternalDataStructures()) {
    return false;
  }
  if (!mixer_.updateInternalDataStructures()) {
    return false;
  }

  // Update the maximum total thrust
  max_thrust_sum_ = 0.;
  for (const auto& [link_name, _] : drone_.prop->rotors) {
    const auto thrust_at_full_thort = drone_.prop->thrustFromThrottle(link_name, tobas::kMaxThrot);
    max_thrust_sum_ += thrust_at_full_thort;
  }

  return true;
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

kdl::Vector ControllerNode::computeEulerError(const kdl::Euler& cur_rpy, const kdl::Euler& tar_rpy) const
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
  resetIntegralErrors();
  return true;
}

bool ControllerNode::headingNaturalFrequencyCb(const double& p)
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
  resetIntegralErrors();
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

bool ControllerNode::maxAttitudeCb(const long& p)
{
  return acc_atti_conv_.setMaxAttitude(tobas_std::deg2rad(p));
}

bool ControllerNode::throttleGainThresholdCb(const long& p)
{
  throttle_gain_thresh_ = static_cast<double>(p) / 100.;
  return true;
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
  auto feedback = std::make_unique<tobas_debug_msgs::MultiRotorControllerFeedback>();
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
    acc_atti_conv_.update(
      odom->frame.M, acc_cmd_->accel, dist_force_W, tar_thrust_, tar_angle_->roll, tar_angle_->pitch);

    // ヨー角はそのまま流す
    tar_angle_->yaw = acc_cmd_->yaw;

    // フィードバックメッセージを埋める
    feedback->target_accel = odom->frame.M.inverse(acc_cmd_->accel);
  }

  // 目標推力が重量の割合で定められた閾値未満のときは，推力が小さいほど姿勢制御の自然周波数が小さくなるように調整する．
  // これで低速域でのジャイロに対する可変ピッチの感度が一定になる (memo: 3-33)
  const auto thrust_thresh = mass_holder_.getMass() * tobas_std::kGravity * throttle_gain_thresh_;
  const auto thrust_ratio = thrust_thresh > 0. ? tar_thrust_ / thrust_thresh : INFINITY;
  const auto thrust_coef = std::min(thrust_ratio, 1.);
  const auto atti_wn = atti_wn_ * thrust_coef;
  const auto head_wn = head_wn_ * thrust_coef;

  // 姿勢制御器
  if (tar_angle_) {
    if (!tar_gyro_) {
      tar_gyro_ = std::make_shared<kdl::Vector>();
    }

    // 現在のオイラー角を計算
    const kdl::Euler cur_rpy(odom->frame.M);

    // PD制御を2段階に分割したときのゲインを計算 (memo: 3-22)
    const auto atti_kp = atti_wn / atti_zeta_ / 2;
    const auto head_kp = head_wn / head_zeta_ / 2;
    const kdl::Vector kp(atti_kp, atti_kp, head_kp);

    // 誤差を計算
    const auto ep = computeEulerError(cur_rpy, *tar_angle_);

    // 浮遊していれば積分誤差を蓄積
    if (!landed_->data) {
      rot_ei_ += ep * dt;
    }

    // 目標オイラー角速度を計算 (接地している場合はI制御は行わない)
    const auto tar_drpy = kp.hadamard(ep) + rot_ki_.hadamard(rot_ei_);

    // オイラー角速度をジャイロに変換
    *tar_gyro_ = eigen::angvelFromEulerrateLocal(tar_drpy.data, cur_rpy.roll, cur_rpy.pitch);

    // フィードバックメッセージを埋める
    feedback->target_angle = *tar_angle_;
    feedback->angle_integral_error = rot_ei_;
  }

  // 角速度制御器
  if (tar_gyro_) {
    // PD制御を2段階に分割したときのゲインを計算 (memo: 3-22)
    const auto atti_kd = atti_wn * atti_zeta_ * 2;
    const auto head_kd = head_wn * head_zeta_ * 2;
    const kdl::Vector kd(atti_kd, atti_kd, head_kd);

    // 目標角加速度を計算
    const auto& cur_gyro = odom->twist.rot;
    const auto tar_dgyro = kd.hadamard(*tar_gyro_ - cur_gyro);

    // プロペラの推力を計算
    const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
    if (!mixer_.solve(js_converter_.getPosition(), cur_gyro, tar_dgyro, tar_thrust_, dist_torque_B)) {
      TOBAS_FATAL("Failed to solve mixing equation.");
      return;
    }

    // 目標推力を発行
    auto thrusts_msg = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    thrusts_msg->header.stamp = odom->header.stamp;
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      thrusts_msg->thrusts.emplace_back();
      thrusts_msg->thrusts.back().link_name = rotor_it.first;
      thrusts_msg->thrusts.back().thrust = mixer_.getThrust(idx);  // 微小値はゼロに固定
    }
    tar_thrusts_pub_->publish(std::move(thrusts_msg));

    // フィードバックメッセージを埋める
    feedback->target_gyro = *tar_gyro_;
    feedback->target_dgyro = tar_dgyro;

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

void ControllerNode::landedCb(const tobas_std_msgs::msg::BoolStamped::ConstSharedPtr& landed)
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
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (fabs(angle_cmd->angle.pitch) > M_PI_2) {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();
  acc_cmd_.reset();

  // コマンドを更新
  tar_angle_ = std::make_shared<kdl::Euler>(angle_cmd->angle);
  tar_thrust_ = max_thrust_sum_ * std::clamp(angle_cmd->throttle, tobas::kMinThrot, tobas::kMaxThrot);
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
  tar_thrust_ = max_thrust_sum_ * std::clamp(rate_cmd->throttle, tobas::kMinThrot, tobas::kMaxThrot);
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

  if (dist_force_sub_ && !dist_force_) {
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
