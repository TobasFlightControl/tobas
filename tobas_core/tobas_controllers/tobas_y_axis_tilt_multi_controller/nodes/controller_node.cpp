#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_constants/node.hpp>
#include <tobas_constants/ros_interface.hpp>
#include <tobas_constants/throttle.hpp>
#include <tobas_constants/time.hpp>
#include <tobas_kdl/tree_mass_holder.hpp>
#include <tobas_node/node.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/command_priority_handler.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>

#include <tobas_command_msgs_adapter/accel_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/angle_throttle_vector.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_pitch_yaw.hpp>
#include <tobas_command_msgs_adapter/rate_throttle_vector.hpp>
#include <tobas_debug_msgs_adapter/multicopter_controller_feedback.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/joint_command_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/landed_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

#include "tobas_y_axis_tilt_multi_controller/mixer.hpp"
#include "tobas_y_axis_tilt_multi_controller/translational_eom.hpp"

namespace tobas
{
namespace y_axis_tilt_multicopter
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

  // Controller
  TranslationalEoM trans_eom_;
  Mixer mixer_;
  double throttle_gain_thresh_;  // [-]
  struct TranslationControlParameters
  {
    double hor_wn, ver_wn;      // [rad/s]
    double hor_zeta, ver_zeta;  // [-]
    double hor_ki, ver_ki;
    double hor_max_i_acc, ver_max_i_acc;
    kdl::Vector ei = kdl::Vector::Zero();
  } trans_ctrl_;
  struct RotationControlParameters
  {
    double atti_wn, head_wn;      // [rad/s]
    double atti_zeta, head_zeta;  // [-]
    double atti_ki, head_ki;
    kdl::Vector ei = kdl::Vector::Zero();
  } rot_ctrl_;

  // Values depending on drone configuration
  double max_thrust_sum_;  // [N]

  // State
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool js_received_ = false;
  bool topics_received_ = false;
  CommandPriorityHandler cmd_priority_handler_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  tobas_msgs::msg::LandedState::ConstSharedPtr landed_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  // Command
  tobas_command_msgs::PosVelAccPitchYaw::UniquePtr pos_cmd_;  // 位置制御の目標値 (世界座標系)
  tobas_command_msgs::AccelPitchYaw::UniquePtr acc_cmd_;      // 加速度制御の目標値 (世界座標系)
  std::unique_ptr<kdl::Rotation> tar_rot_;                    // 目標姿勢 (機体座標系)
  std::unique_ptr<kdl::Vector> tar_gyro_;                     // 目標角速度 (機体座標系)
  kdl::Vector tar_dgyro_;                                     // 目標角加速度 (機体座標系)
  double ux_ = 0., uz_ = 0.;                                  // 目標推力 (機体座標系)

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> tar_angles_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::MulticopterControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::LandedState> landed_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::PosVelAccPitchYaw> pos_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::AccelPitchYaw> acc_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::AngleThrottleVector> angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::RateThrottleVector> rate_cmd_sub_;

  // Timers
  ros2::TimerPtr check_topics_timer_;

  bool updateInternalDataStructures();
  bool isCommandAccepted(const tobas_command_msgs::msg::Priority& priority);

  // Parameter callbacks
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
  bool throttleGainThresholdCb(const double& p);

  // Topic callbacks
  void droneCb(const Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void landedCb(const tobas_msgs::msg::LandedState::ConstSharedPtr& landed);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness);
  void positionCommandCb(const tobas_command_msgs::PosVelAccPitchYaw::ConstSharedPtr& pos_cmd);
  void accelCommandCb(const tobas_command_msgs::AccelPitchYaw::ConstSharedPtr& acc_cmd);
  void angleCommandCb(const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr& angle_cmd);
  void rateCommandCb(const tobas_command_msgs::RateThrottleVector::ConstSharedPtr& rate_cmd);

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
  addDynamicDoubleParam("throttle_gain_threshold", &self::throttleGainThresholdCb, this, 1., 70, 0, 100, " %");

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(topic::kRotorThrustsCmd);
  tar_angles_pub_ = createPublisher<tobas_msgs::msg::JointCommandArray>(tobas::topic::kJointPosCmd);
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
  pos_cmd_sub_ = createSubscriber(topic::kPosVelAccPitchYawCmd, &self::positionCommandCb, this);
  acc_cmd_sub_ = createSubscriber(topic::kAccelPitchYawCmd, &self::accelCommandCb, this);
  angle_cmd_sub_ = createSubscriber(topic::kAngleThrotVectorCmd, &self::angleCommandCb, this);
  rate_cmd_sub_ = createSubscriber(topic::kRateThrotVectorCmd, &self::rateCommandCb, this);

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
  trans_ctrl_.hor_wn = p;
  return true;
}

bool ControllerNode::horizontalDampingRatioCb(const double& p)
{
  trans_ctrl_.hor_zeta = p;
  return true;
}

bool ControllerNode::horizontalIGainCb(const double& p)
{
  trans_ctrl_.hor_ki = p;
  return true;
}

bool ControllerNode::horizontalIMaxAccelCb(const double& p)
{
  trans_ctrl_.hor_max_i_acc = p;
  return true;
}

bool ControllerNode::verticalNaturalFreqCb(const double& p)
{
  trans_ctrl_.ver_wn = p;
  return true;
}

bool ControllerNode::verticalDampingRatioCb(const double& p)
{
  trans_ctrl_.ver_zeta = p;
  return true;
}

bool ControllerNode::verticalIGainCb(const double& p)
{
  trans_ctrl_.ver_ki = p;
  return true;
}

bool ControllerNode::verticalIMaxAccelCb(const double& p)
{
  trans_ctrl_.ver_max_i_acc = p;
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

bool ControllerNode::attitudeIGainCb(const double& p)
{
  rot_ctrl_.atti_ki = p;
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

bool ControllerNode::headingIGainCb(const double& p)
{
  rot_ctrl_.head_ki = p;
  return true;
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

  // エイリアス
  const auto& cur_pos_W = odom->frame.p;
  const auto& cur_rot = odom->frame.M;
  const auto& cur_vel_B = odom->twist.vel;
  const auto& cur_gyro_B = odom->twist.rot;

  // 目標推力が重量の割合で定められた閾値未満のときは，推力が小さいほど制御器の自然周波数が小さくなるように調整する．
  const auto tar_thrust = math::norm(ux_, uz_);
  const auto thrust_thresh = mass_holder_.getMass() * tbs::kGravity * throttle_gain_thresh_;
  const auto land_suspect = (tar_thrust < thrust_thresh);
  const auto gain_throt = land_suspect ? tar_thrust / thrust_thresh : 1.;

  // 位置制御器
  if (pos_cmd_) {
    if (!acc_cmd_) {
      acc_cmd_ = std::make_unique<tobas_command_msgs::AccelPitchYaw>();
    }

    // ゲインを決定
    const auto hor_wn = trans_ctrl_.hor_wn * gain_throt;
    const auto ver_wn = trans_ctrl_.ver_wn;  // 垂直方向はスロットルしない
    const auto hor_kp = math::sqr(hor_wn);
    const auto ver_kp = math::sqr(ver_wn);
    const auto hor_ki = trans_ctrl_.hor_ki * gain_throt;
    const auto ver_ki = trans_ctrl_.ver_ki;  // 垂直方向はスロットルしない
    const auto hor_kd = 2 * trans_ctrl_.hor_zeta * hor_wn;
    const auto ver_kd = 2 * trans_ctrl_.ver_zeta * ver_wn;
    const kdl::Vector kp(hor_kp, hor_kp, ver_kp);
    const kdl::Vector ki(hor_ki, hor_ki, ver_ki);
    const kdl::Vector kd(hor_kd, hor_kd, ver_kd);

    // 誤差を計算
    const auto ep = pos_cmd_->pos - cur_pos_W;
    const auto ed = pos_cmd_->vel - cur_rot * cur_vel_B;

    // 浮遊していれば積分誤差を蓄積
    if (!land_suspect) {
      assert(trans_ctrl_.hor_ki > 0.);
      assert(trans_ctrl_.ver_ki > 0.);
      const auto hor_max_ei = trans_ctrl_.hor_max_i_acc / trans_ctrl_.hor_ki;
      const auto ver_max_ei = trans_ctrl_.ver_max_i_acc / trans_ctrl_.ver_ki;
      const kdl::Vector max_ei(hor_max_ei, hor_max_ei, ver_max_ei);
      const auto next_ei = trans_ctrl_.ei + ep * dt;
      trans_ctrl_.ei = next_ei.clamp(-max_ei, max_ei);
    }

    // 目標加速度を計算
    acc_cmd_->accel = kp.hadamard(ep) + ki.hadamard(trans_ctrl_.ei) + kd.hadamard(ed);

    // ピッチ，ヨー角はそのまま流す
    acc_cmd_->pitch = pos_cmd_->pitch;
    acc_cmd_->yaw = pos_cmd_->yaw;

    // フィードバックメッセージを埋める
    feedback->target_position = pos_cmd_->pos;
    feedback->target_velocity = cur_rot.inverse(pos_cmd_->vel);
    feedback->position_integral_error = trans_ctrl_.ei;
  }

  // 加速度制御器
  if (acc_cmd_) {
    if (!tar_rot_) {
      tar_rot_ = std::make_unique<kdl::Rotation>();
    }

    // 推力と目標姿勢を計算
    const auto& dist_force_W = do_dist_comp_trans_ ? dist_force_->wrench.force : kdl::Vector::Zero();
    if (!trans_eom_.solve(acc_cmd_->accel, acc_cmd_->pitch, acc_cmd_->yaw, dist_force_W, ux_, uz_, *tar_rot_)) {
      TOBAS_FATAL("Failed to solve the translational EoM.");
      return;
    }

    // フィードバックメッセージを埋める
    feedback->target_accel = cur_rot.inverse(acc_cmd_->accel);
  }

  // 姿勢制御器
  if (tar_rot_) {
    if (!tar_gyro_) {
      tar_gyro_ = std::make_unique<kdl::Vector>();
    }

    // ゲインを決定
    const auto atti_wn = rot_ctrl_.atti_wn * gain_throt;
    const auto head_wn = rot_ctrl_.head_wn * gain_throt;
    const auto atti_angle_gain = atti_wn / rot_ctrl_.atti_zeta / 2;
    const auto head_angle_gain = head_wn / rot_ctrl_.head_zeta / 2;
    const auto atti_ki = rot_ctrl_.atti_ki * gain_throt;
    const auto head_ki = rot_ctrl_.head_ki * gain_throt;
    const kdl::Vector angle_gain(atti_angle_gain, atti_angle_gain, head_angle_gain);
    const kdl::Vector ki(atti_ki, atti_ki, head_ki);

    // 回転誤差を計算
    const auto ep = (cur_rot.inverse() * *tar_rot_).getRot();

    // 浮遊していれば積分誤差を蓄積
    if (!land_suspect) {
      for (int i = 0; i < 3; ++i) {
        if (ki(i) > 0.) {
          rot_ctrl_.ei(i) += ep(i) * dt;
        }
        else {
          rot_ctrl_.ei(i) = 0.;
        }
      }
    }

    // 目標角速度を計算
    *tar_gyro_ = angle_gain.hadamard(ep) + ki.hadamard(rot_ctrl_.ei);

    // フィードバックメッセージを埋める
    feedback->target_angle = kdl::Euler(*tar_rot_);
    feedback->angle_integral_error = rot_ctrl_.ei;
  }

  if (tar_gyro_) {
    // 角速度制御器
    {
      // ゲインを決定
      const auto atti_wn = rot_ctrl_.atti_wn * gain_throt;
      const auto head_wn = rot_ctrl_.head_wn * gain_throt;
      const auto atti_rate_gain = atti_wn * rot_ctrl_.atti_zeta * 2;
      const auto head_rate_gain = head_wn * rot_ctrl_.head_zeta * 2;
      const kdl::Vector rate_gain(atti_rate_gain, atti_rate_gain, head_rate_gain);

      // 目標角加速度を計算
      tar_dgyro_ = rate_gain.hadamard(*tar_gyro_ - cur_gyro_B);

      // フィードバックメッセージを埋める
      feedback->target_gyro = *tar_gyro_;
    }

    // ミキサー
    {
      const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
      if (!mixer_.solve(js_converter_.getPosition(), cur_gyro_B, tar_dgyro_, ux_, uz_, dist_torque_B)) {
        TOBAS_FATAL("Failed to solve the mixing equation.");
        return;
      }

      // フィードバックメッセージを埋める
      feedback->target_dgyro = tar_dgyro_;
    }

    // 目標推力を発行
    auto tar_thrusts = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    tar_thrusts->header.stamp = odom->header.stamp;
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      tar_thrusts->thrusts.emplace_back();
      tar_thrusts->thrusts.back().link_name = rotor_it.first;
      tar_thrusts->thrusts.back().thrust = mixer_.getThrust(idx);
    }
    tar_thrusts_pub_->publish(std::move(tar_thrusts));

    // 目標チルト角を発行
    auto tar_angles = std::make_unique<tobas_msgs::msg::JointCommandArray>();
    tar_angles->header.stamp = odom->header.stamp;
    for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
      const auto& rotor = rotor_it.second;
      if (rotor->tilt_joint_name.empty()) {
        continue;
      }
      tar_angles->commands.emplace_back();
      tar_angles->commands.back().name = rotor->tilt_joint_name;
      tar_angles->commands.back().data = mixer_.getTiltAngle(idx);
    }
    tar_angles_pub_->publish(std::move(tar_angles));

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
    trans_ctrl_.ei.setZero();
    rot_ctrl_.ei.setZero();

    pos_cmd_.reset();
    acc_cmd_.reset();
    tar_rot_.reset();
    tar_gyro_.reset();

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

void ControllerNode::positionCommandCb(const tobas_command_msgs::PosVelAccPitchYaw::ConstSharedPtr& pos_cmd)
{
  if (!isCommandAccepted(pos_cmd->priority)) {
    return;
  }

  // コマンドを更新
  pos_cmd_ = std::make_unique<tobas_command_msgs::PosVelAccPitchYaw>(*pos_cmd);
}

void ControllerNode::accelCommandCb(const tobas_command_msgs::AccelPitchYaw::ConstSharedPtr& acc_cmd)
{
  if (!isCommandAccepted(acc_cmd->priority)) {
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();

  // コマンドを更新
  acc_cmd_ = std::make_unique<tobas_command_msgs::AccelPitchYaw>(*acc_cmd);
}

void ControllerNode::angleCommandCb(const tobas_command_msgs::AngleThrottleVector::ConstSharedPtr& angle_cmd)
{
  if (!isCommandAccepted(angle_cmd->priority)) {
    return;
  }

  // Check command range
  if (std::abs(angle_cmd->angle.roll) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (std::abs(angle_cmd->angle.pitch) > M_PI_2) {
    TOBAS_WARN_THROTTLE(kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();
  acc_cmd_.reset();

  // コマンドを更新
  tar_rot_ = std::make_unique<kdl::Rotation>(angle_cmd->angle.toRotation());
  const auto tar_thrust = max_thrust_sum_ * std::clamp(angle_cmd->throttle, kMinThrot, kMaxThrot);
  ux_ = tar_thrust * sin(angle_cmd->thrust_angle);
  uz_ = tar_thrust * cos(angle_cmd->thrust_angle);
}

void ControllerNode::rateCommandCb(const tobas_command_msgs::RateThrottleVector::ConstSharedPtr& rate_cmd)
{
  if (!isCommandAccepted(rate_cmd->priority)) {
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();
  acc_cmd_.reset();
  tar_rot_.reset();

  // コマンドを更新
  tar_gyro_ = std::make_unique<kdl::Vector>(rate_cmd->rate);
  const auto tar_thrust = max_thrust_sum_ * std::clamp(rate_cmd->throttle, kMinThrot, kMaxThrot);
  ux_ = tar_thrust * sin(rate_cmd->thrust_angle);
  uz_ = tar_thrust * cos(rate_cmd->thrust_angle);
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
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::y_axis_tilt_multicopter::ControllerNode)
