#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_pose_pid/euler_pi.hpp>
#include <tobas_drone_tools/mr_accel_attitude_converter.hpp>
#include <tobas_drone_tools/mr_mixer_qp.hpp>

#include <tobas_std_msgs/msg/bool_stamped.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_command_msgs_adapter/rate_throttle.hpp>
#include <tobas_command_msgs_adapter/angle_throttle.hpp>
#include <tobas_command_msgs_adapter/accel_yaw.hpp>
#include <tobas_command_msgs_adapter/pos_vel_yaw.hpp>
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
  tobas::RotorAxisExtractor z_rotors_;

  // Static parameters
  bool do_dist_comp_trans_;
  bool do_dist_comp_rot_;

  // Controller
  tobas::PositionPID pos_pid_;
  tobas::EulerPI rot_pi_;
  tobas::AccelAttitudeConverter acc_atti_conv_;
  tobas::MultiRotorMixer_QP mixer_;
  double atti_wn_, head_wn_;      // [rad/s]
  double atti_zeta_, head_zeta_;  // [-]
  kdl::Vector gyro_gain_;

  // State
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool js_received_ = false;
  tobas::CommandLevelHandler cmd_level_handler_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  tobas_std_msgs::msg::BoolStamped::ConstSharedPtr landed_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  // Command
  tobas_command_msgs::PosVelYaw::SharedPtr pos_cmd_;  // 位置制御の目標値 (世界座標系)
  tobas_command_msgs::AccelYaw::SharedPtr acc_cmd_;   // 加速度制御の目標値 (世界座標系)
  shared_ptr<kdl::Euler> tar_angle_;                  // 目標オイラー角 (世界座標系)
  shared_ptr<kdl::Vector> tar_gyro_;                  // 目標ジャイロ (機体座標系P)
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
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::PosVelYaw> pos_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::AccelYaw> acc_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::AngleThrottle> angle_cmd_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::RateThrottle> rate_cmd_sub_;

  bool updateInternalDataStructures();
  bool isReadyToControl();
  bool updateAttitudePDGain();
  bool updateHeadingPDGain();
  void resetCommands();
  void resetIntegralGains();

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
  bool maxAttitudeCb(const double& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void landedCb(const tobas_std_msgs::msg::BoolStamped::ConstSharedPtr& landed);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness);
  void positionCommandCb(const tobas_command_msgs::PosVelYaw::ConstSharedPtr& pos_cmd);
  void accelCommandCb(const tobas_command_msgs::AccelYaw::ConstSharedPtr& acc_cmd);
  void angleCommandCb(const tobas_command_msgs::AngleThrottle::ConstSharedPtr& angle_cmd);
  void rateCommandCb(const tobas_command_msgs::RateThrottle::ConstSharedPtr& rate_cmd);
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::node::kController, options),
    js_converter_(tree_),
    z_rotors_(drone_, tobas::Z_POSITIVE),
    acc_atti_conv_(tree_),
    mixer_(drone_, tree_)
{
  // Get static parameters
  do_dist_comp_trans_ = getBoolParam("do_disturbance_compensation_translation");
  do_dist_comp_rot_ = getBoolParam("do_disturbance_compensation_rotation");

  // Iゲインは1~2秒で位置の補正が感じられるくらいに設定するのが良いらしい (GPT o1)
  const auto default_trans_i_gain = do_dist_comp_trans_ ? 0. : 0.1;
  const auto default_rot_i_gain = do_dist_comp_rot_ ? 0. : 1.;

  // Register dynamic parameters
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFrequencyCb, this, 1., 0.1, 5.);
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFrequencyCb, this, 10., 1., 50.);
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFrequencyCb, this, 5., 0.1, 25.);
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, default_trans_i_gain, 0., 1.);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, default_trans_i_gain, 0., 1.);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, default_rot_i_gain, 0., 10.);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, default_rot_i_gain, 0., 10.);
  addDynamicDoubleParam("max_horizontal_accel", &self::maxHorizontalAccelCb, this, 8., 1., 20.);
  addDynamicDoubleParam("max_vertical_accel", &self::maxVerticalAccelCb, this, 4., 1., 10.);
  addDynamicDoubleParam("max_attitude", &self::maxAttitudeCb, this, M_PI / 3, 0., M_PI_2 - 1e-3);

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(tobas::kRotorThrustsCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MultiRotorControllerFeedback>(tobas::kMRCtrlFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKdlTreeTopic, &self::treeCb, this, true, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  dist_force_sub_ = createSubscriber(tobas::kDisturbanceForceTopic, &self::disturbanceForceCb, this);
  landed_sub_ = createSubscriber(tobas::kLandedTopic, &self::landedCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  rotor_liveliness_sub_ = createSubscriber(tobas::kRotorLivelinessTopic, &self::rotorLivelinessCb, this);
  pos_cmd_sub_ = createSubscriber(tobas::kPosVelYawCmdTopic, &self::positionCommandCb, this);
  acc_cmd_sub_ = createSubscriber(tobas::kAccelYawCmdTopic, &self::accelCommandCb, this);
  angle_cmd_sub_ = createSubscriber(tobas::kAngleThrottleCmdTopic, &self::angleCommandCb, this);
  rate_cmd_sub_ = createSubscriber(tobas::kRateThrottleCmdTopic, &self::rateCommandCb, this);
}

bool ControllerNode::updateInternalDataStructures()
{
  if (!z_rotors_.updateInternalDataStructures())
    return false;
  if (!js_converter_.updateInternalDataStructures())
    return false;
  if (!acc_atti_conv_.updateInternalDataStructures())
    return false;
  if (!mixer_.updateInternalDataStructures())
    return false;

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
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kKdlTreeTopic, "\".");
    return false;
  }

  if (!odom_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kOdometryTopic, "\".");
    return false;
  }

  if (!dist_force_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kDisturbanceForceTopic, "\".");
    return false;
  }

  if (js_sub_ && !js_received_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kJointStatesTopic, "\".");
    return false;
  }

  if (!landed_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kLandedTopic, "\".");
    return false;
  }

  if (!arming_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kArmingTopic, "\".");
    return false;
  }

  if (!arming_->data)
    return false;

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

void ControllerNode::resetCommands()
{
  pos_cmd_.reset();
  acc_cmd_.reset();
  tar_angle_.reset();
  tar_gyro_.reset();
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

bool ControllerNode::maxAttitudeCb(const double& p)
{
  return acc_atti_conv_.setMaxAttitude(p);
}

void ControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;

  if (drone->hasServoJoint())
    js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::jointStateCb, this);
  else
    js_sub_.reset();

  if (tree_received_)
  {
    if (!updateInternalDataStructures())
    {
      TOBAS_FATAL("Error occured while updating internal data structures.");
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
    if (!updateInternalDataStructures())
    {
      TOBAS_FATAL("Error occured while updating internal data structures.");
      return;
    }
  }

  tree_received_ = true;
}

void ControllerNode::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
{
  if (!odom_)
  {
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
  if (pos_cmd_)
  {
    if (!acc_cmd_)
      acc_cmd_ = std::make_shared<tobas_command_msgs::AccelYaw>();

    // 世界座標系から見た現在の位置速度
    const auto& cur_pos_W = odom->frame.p;
    const auto cur_vel_W = odom->frame.M * odom->twist.vel;

    // 目標加速度を計算
    // 接地している場合はI制御は行わない
    if (landed_->data)
      acc_cmd_->accel = pos_pid_.updatePD(cur_pos_W, cur_vel_W, pos_cmd_->pos, pos_cmd_->vel);
    else
      acc_cmd_->accel = pos_pid_.updatePID(cur_pos_W, cur_vel_W, pos_cmd_->pos, pos_cmd_->vel, dt);

    // ヨー角はそのまま流す
    acc_cmd_->yaw = pos_cmd_->yaw;

    // フィードバックメッセージを埋める
    feedback->target_position = pos_cmd_->pos;
    feedback->target_velocity = odom->frame.M.inverse(pos_cmd_->vel);
    feedback->position_integral_error = pos_pid_.getIntegralError();
  }

  // 加速度制御器
  if (acc_cmd_)
  {
    if (!tar_angle_)
      tar_angle_ = std::make_shared<kdl::Euler>();

    // 推力和と目標姿勢を計算
    const auto& dist_force_W = do_dist_comp_trans_ ? dist_force_->wrench.force : kdl::Vector::Zero();
    acc_atti_conv_.update(
      odom->frame.M, acc_cmd_->accel, dist_force_W, tar_thrust_, tar_angle_->roll, tar_angle_->pitch);

    // ヨー角はそのまま流す
    tar_angle_->yaw = acc_cmd_->yaw;

    // フィードバックメッセージを埋める
    feedback->target_accel = odom->frame.M.inverse(acc_cmd_->accel);
  }

  // 姿勢制御器
  if (tar_angle_)
  {
    if (!tar_gyro_)
      tar_gyro_ = std::make_shared<kdl::Vector>();

    // 現在のオイラー角を計算
    const kdl::Euler cur_rpy(odom->frame.M);

    // 目標角速度を計算
    // 接地している場合はI制御は行わない
    if (landed_->data)
      *tar_gyro_ = rot_pi_.updateP(cur_rpy, *tar_angle_);
    else
      *tar_gyro_ = rot_pi_.updatePI(cur_rpy, *tar_angle_, dt);

    // フィードバックメッセージを埋める
    feedback->target_angle = *tar_angle_;
    feedback->angle_integral_error = rot_pi_.getIntegralError();
  }

  // 角速度制御器
  if (tar_gyro_)
  {
    // 目標角加速度を計算
    const auto& cur_gyro = odom->twist.rot;
    const auto tar_dgyro = gyro_gain_.hadamard(*tar_gyro_ - cur_gyro);

    // プロペラの推力を計算
    const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
    if (!mixer_.solve(js_converter_.getPosition(), cur_gyro, tar_dgyro, tar_thrust_, dist_torque_B))
    {
      TOBAS_FATAL("Failed to solve mixing equation.");
      return;
    }
    const auto& thrusts = mixer_.getThrusts();

    // 目標推力を発行
    auto thrusts_msg = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    thrusts_msg->header.stamp = odom->header.stamp;
    for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors))
    {
      thrusts_msg->thrusts.emplace_back();
      thrusts_msg->thrusts.back().link_name = rotor_it.first;
      thrusts_msg->thrusts.back().thrust = thrusts(idx);
    }
    tar_thrusts_pub_->publish(move(thrusts_msg));

    // フィードバックメッセージを埋める
    feedback->target_gyro = *tar_gyro_;
    feedback->target_dgyro = tar_dgyro;

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
  if (js_converter_.convert(*js) < 0)
  {
    TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());
    return;
  }

  js_received_ = true;
}

void ControllerNode::landedCb(const tobas_std_msgs::msg::BoolStamped::ConstSharedPtr& landed)
{
  if (landed->data)
    resetIntegralGains();

  landed_ = landed;
}

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  if (arming_ && arming_->data && !arming->data)
  {
    resetCommands();
    resetIntegralGains();
    TOBAS_INFO("Controller is reset.");
  }

  arming_ = arming;
}

void ControllerNode::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness)
{
  for (const auto& data : rotor_liveliness->data)
    if (!mixer_.setRotorLiveliness(data.link_name, data.alive))
      TOBAS_ERROR("Failed to set the liveliness of rotor \"", data.link_name, "\".");
}

void ControllerNode::positionCommandCb(const tobas_command_msgs::PosVelYaw::ConstSharedPtr& pos_cmd)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(pos_cmd->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // コマンドを更新
  pos_cmd_ = std::make_shared<tobas_command_msgs::PosVelYaw>(*pos_cmd);
}

void ControllerNode::accelCommandCb(const tobas_command_msgs::AccelYaw::ConstSharedPtr& acc_cmd)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(acc_cmd->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();

  // コマンドを更新
  acc_cmd_ = std::make_shared<tobas_command_msgs::AccelYaw>(*acc_cmd);
}

void ControllerNode::angleCommandCb(const tobas_command_msgs::AngleThrottle::ConstSharedPtr& angle_cmd)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(angle_cmd->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // Check command range
  if (angle_cmd->angle.roll <= -M_PI_2 || M_PI_2 <= angle_cmd->angle.roll)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (angle_cmd->angle.pitch <= -M_PI_2 || M_PI_2 <= angle_cmd->angle.pitch)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }
  if (angle_cmd->throttle < tobas::kMinThrot || tobas::kMaxThrot < angle_cmd->throttle)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target throttle is invalid.");
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();
  acc_cmd_.reset();

  // コマンドを更新
  tar_angle_ = std::make_shared<kdl::Euler>(angle_cmd->angle);
  tar_thrust_ = z_rotors_.maxThrustSum() * angle_cmd->throttle;
}

void ControllerNode::rateCommandCb(const tobas_command_msgs::RateThrottle::ConstSharedPtr& rate_cmd)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(rate_cmd->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // Check command range
  if (rate_cmd->throttle < tobas::kMinThrot || tobas::kMaxThrot < rate_cmd->throttle)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target throttle is invalid.");
    return;
  }

  // 外側の制御を止める
  pos_cmd_.reset();
  acc_cmd_.reset();
  tar_angle_.reset();

  // コマンドを更新
  tar_gyro_ = std::make_shared<kdl::Vector>(rate_cmd->rate);
  tar_thrust_ = z_rotors_.maxThrustSum() * rate_cmd->throttle;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
