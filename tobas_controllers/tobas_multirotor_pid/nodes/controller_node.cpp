#include <ranges>

#include <tobas_algorithm/core.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_drone_tools/mr_accel_attitude_converter.hpp>
#include <tobas_drone_tools/mr_mixer_qp.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_command_msgs/msg/rate_throttle.hpp>
#include <tobas_command_msgs/msg/angle_throttle.hpp>
#include <tobas_command_msgs_adapter/pos_vel_acc_yaw.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_debug_msgs_adapter/multi_rotor_controller_feedback.hpp>

using namespace std;
using namespace Eigen;

struct AngleThrust
{
  kdl::Euler rpy;  // [rad]
  double thrust;   // [N]
};

struct RateThrust
{
  kdl::Euler drpy;  // [rad/s]
  double thrust;    // [N]
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

  tobas::TreeJointStateConverter js_converter_;
  tobas::RotorAxisExtractor z_rotors_;

  // Static parameters
  bool do_dist_comp_trans_;
  bool do_dist_comp_rot_;

  // Controller
  tobas::PositionPID pos_pid_;
  tobas::AccelAttitudeConverter acc_atti_conv_;
  tobas::MultiRotorMixer_QP mixer_;
  double atti_wn_, head_wn_;      // [rad/s]
  double atti_zeta_, head_zeta_;  // [-]

  // State
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool js_received_ = false;
  tobas::CommandLevelHandler cmd_level_handler_;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  // Command
  tobas_command_msgs::PosVelAccYaw::SharedPtr pvay_cmd_;  // 位置制御の目標値 (世界座標系)
  shared_ptr<AngleThrust> angle_cmd_;                     // 姿勢制御の目標値
  shared_ptr<RateThrust> rate_cmd_;                       // 角速度制御の目標値

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::MultiRotorControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::PosVelAccYaw> pvay_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::AngleThrottle> angle_throt_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::msg::RateThrottle> rate_throt_sub_;

  bool updateInternalDataStructures();
  bool isReadyToControl();

  bool horizontalNaturalFrequencyCb(const double& p);
  bool horizontalDampingRatioCb(const double& p);
  bool horizontalIGainCb(const double& p);
  bool verticalNaturalFrequencyCb(const double& p);
  bool verticalDampingRatioCb(const double& p);
  bool verticalIGainCb(const double& p);
  bool attitudeNaturalFrequencyCb(const double& p);
  bool attitudeDampingRatioCb(const double& p);
  bool headingNaturalFrequencyCb(const double& p);
  bool headingDampingRatioCb(const double& p);
  bool maxHorizontalAccelCb(const double& p);
  bool maxVerticalAccelCb(const double& p);
  bool maxAttitudeCb(const double& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness);
  void posVelAccYawCmdCb(const tobas_command_msgs::PosVelAccYaw::ConstSharedPtr& pvay);
  void angleThrustCmdCb(const tobas_command_msgs::msg::AngleThrottle::ConstSharedPtr& angle_throt);
  void rateThrustCmdCb(const tobas_command_msgs::msg::RateThrottle::ConstSharedPtr& rate_throt);
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::node::kController, options),
    js_converter_(tree_),
    z_rotors_(drone_, tobas::Z_POSITIVE),
    acc_atti_conv_(tree_),
    mixer_(drone_, tree_)
{
  // Get static parameters
  do_dist_comp_trans_ = getBoolParam("do_disturbance_compensation_translation", true);
  do_dist_comp_rot_ = getBoolParam("do_disturbance_compensation_rotation", false);

  // Register dynamic parameters
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFrequencyCb, this, 1., 0.1, 5.);
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFrequencyCb, this, 10., 1., 50.);
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFrequencyCb, this, 5., 0.1, 25.);
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0., 0., 10.);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0., 0., 10.);
  addDynamicDoubleParam("max_horizontal_accel", &self::maxHorizontalAccelCb, this, 8., 1., 20.);
  addDynamicDoubleParam("max_vertical_accel", &self::maxVerticalAccelCb, this, 4., 1., 10.);
  addDynamicDoubleParam("max_attitude", &self::maxAttitudeCb, this, M_PI / 3, 0., M_PI_2 - 1e-3);

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(tobas::kRotorThrustsCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MultiRotorControllerFeedback>(tobas::kMRCtrlFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  dist_force_sub_ = createSubscriber(tobas::kDisturbanceForceTopic, &self::disturbanceForceCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  rotor_liveliness_sub_ = createSubscriber(tobas::kRotorLivelinessTopic, &self::rotorLivelinessCb, this);
  pvay_sub_ = createSubscriber(tobas::kPosVelAccYawCmdTopic, &self::posVelAccYawCmdCb, this);
  angle_throt_sub_ = createSubscriber(tobas::kAngleThrottleCmdTopic, &self::angleThrustCmdCb, this);
  rate_throt_sub_ = createSubscriber(tobas::kRateThrottleCmdTopic, &self::rateThrustCmdCb, this);
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
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kKDLTreeTopic, "\".");
    return false;
  }

  if (odom_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kOdometryTopic, "\".");
    return false;
  }

  if (dist_force_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kDisturbanceForceTopic, "\".");
    return false;
  }

  if (js_sub_ != nullptr && !js_received_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kJointStatesTopic, "\".");
    return false;
  }

  if (arming_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kArmingTopic, "\".");
    return false;
  }

  if (!arming_->data)
    return false;

  return true;
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
    js_sub_ = nullptr;

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
  if (odom_ == nullptr)
  {
    odom_ = odom;
    return;
  }

  // 経過時間を計算してオドメトリを更新
  const auto dt = (odom->header.stamp - odom_->header.stamp).seconds();
  odom_ = odom;

  // 現在のオイラー角を計算
  const kdl::Euler cur_rpy(odom->frame.M);

  // フィードバックメッセージを作成
  auto feedback_msg = std::make_unique<tobas_debug_msgs::MultiRotorControllerFeedback>();
  feedback_msg->header.stamp = odom->header.stamp;

  // 位置制御器
  if (pvay_cmd_ != nullptr)
  {
    if (angle_cmd_ == nullptr)
      angle_cmd_ = std::make_shared<AngleThrust>();

    // 世界座標系から見た現在の位置速度
    const auto& cur_pos_W = odom->frame.p;
    const auto cur_vel_W = odom->frame.M * odom->twist.vel;

    // 目標加速度を計算
    const auto tar_acc_fb = pos_pid_.update(cur_pos_W, cur_vel_W, pvay_cmd_->pos, pvay_cmd_->vel, dt);
    const auto tar_acc = pvay_cmd_->acc + tar_acc_fb;

    // 推力和と目標姿勢を計算
    const auto& dist_force_W = do_dist_comp_trans_ ? dist_force_->wrench.force : kdl::Vector::Zero();
    acc_atti_conv_.update(
      odom->frame.M, tar_acc, dist_force_W, angle_cmd_->thrust, angle_cmd_->rpy.roll, angle_cmd_->rpy.pitch);

    // ヨー角はそのまま流す
    angle_cmd_->rpy.yaw = pvay_cmd_->yaw;

    // フィードバックメッセージを埋める
    feedback_msg->target_position = pvay_cmd_->pos;
    feedback_msg->target_velocity = pvay_cmd_->vel;
    feedback_msg->target_acceleration = tar_acc;
    feedback_msg->position_integral_error = pos_pid_.integralError();
  }

  // 姿勢制御器
  if (angle_cmd_ != nullptr)
  {
    if (rate_cmd_ == nullptr)
      rate_cmd_ = std::make_shared<RateThrust>();

    // PD制御を2段階に分割したときのゲインを計算 (memo: 3-22)
    const auto atti_gain = atti_wn_ / atti_zeta_ / 2;
    const auto head_gain = head_wn_ / head_zeta_ / 2;

    // 目標角速度を計算
    rate_cmd_->drpy.roll = atti_gain * algo::wrapPi(angle_cmd_->rpy.roll - cur_rpy.roll);
    rate_cmd_->drpy.pitch = atti_gain * algo::wrapPi(angle_cmd_->rpy.pitch - cur_rpy.pitch);
    rate_cmd_->drpy.yaw = head_gain * algo::wrapPi(angle_cmd_->rpy.yaw - cur_rpy.yaw);

    // 目標推力はそのまま流す
    rate_cmd_->thrust = angle_cmd_->thrust;

    // フィードバックメッセージを埋める
    feedback_msg->target_angle = angle_cmd_->rpy;
  }

  // 角速度制御器
  if (rate_cmd_ != nullptr)
  {
    // PD制御を2段階に分割したときのゲインを計算 (memo: 3-22)
    const auto atti_gain = 2 * atti_wn_ * atti_zeta_;
    const auto head_gain = 2 * head_wn_ * head_zeta_;

    // 現在のオイラーレートを計算
    const auto cur_drpy = eigen::eulerrateFromAngvelLocal(odom->twist.rot.data, cur_rpy.roll, cur_rpy.pitch);
    const auto cur_droll = cur_drpy.x();
    const auto cur_dpitch = cur_drpy.y();
    const auto cur_dyaw = cur_drpy.z();

    // 目標角加速度を計算
    Vector3d tar_ddrpy;
    tar_ddrpy.x() = atti_gain * (rate_cmd_->drpy.roll - cur_droll);
    tar_ddrpy.y() = atti_gain * (rate_cmd_->drpy.pitch - cur_dpitch);
    tar_ddrpy.z() = head_gain * (rate_cmd_->drpy.yaw - cur_dyaw);
    const auto tar_dgyro = eigen::angaccFromEuleraccLocal(cur_rpy.roll, cur_rpy.pitch, cur_drpy, tar_ddrpy);

    // プロペラの推力を計算
    const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
    if (!mixer_.solve(js_converter_.getPosition(), odom->twist.rot, tar_dgyro, rate_cmd_->thrust, dist_torque_B))
    {
      TOBAS_FATAL("Failed to solve mixing equation.");
      return;
    }
    const auto& thrusts = mixer_.getThrusts();

    // 目標回転数を発行
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
    feedback_msg->target_angle_rate = rate_cmd_->drpy;
    feedback_msg->target_thrust = rate_cmd_->thrust;
  }

  // フィードバックメッセージを発行
  feedback_pub_->publish(move(feedback_msg));
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

void ControllerNode::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  // Disarm時にコマンドをリセットする．でないと再度アームした時に前回のコマンドでモータが回り始めてしまう．
  if (arming_ != nullptr && arming_->data && !arming->data)
  {
    pvay_cmd_ = nullptr;
    angle_cmd_ = nullptr;
    rate_cmd_ = nullptr;
    TOBAS_INFO("Command is reset.");
  }

  arming_ = arming;
}

void ControllerNode::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness)
{
  for (const auto& data : rotor_liveliness->data)
    if (!mixer_.setRotorLiveliness(data.link_name, data.alive))
      TOBAS_ERROR("Failed to set the liveliness of rotor \"", data.link_name, "\".");
}

void ControllerNode::posVelAccYawCmdCb(const tobas_command_msgs::PosVelAccYaw::ConstSharedPtr& pvay)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(pvay->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // コマンドを更新
  pvay_cmd_ = std::make_shared<tobas_command_msgs::PosVelAccYaw>(*pvay);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_command_msgs::msg::FrameId::WORLD, odom_->frame.M, *pvay_cmd_))
  {
    TOBAS_ERROR("Failed to change command frame. Probably the frame ID is invalid.");
    pvay_cmd_ = nullptr;
    return;
  }
}

void ControllerNode::angleThrustCmdCb(const tobas_command_msgs::msg::AngleThrottle::ConstSharedPtr& angle_throt)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(angle_throt->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // Check command range
  if (angle_throt->roll <= -M_PI_2 || M_PI_2 <= angle_throt->roll)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (angle_throt->pitch <= -M_PI_2 || M_PI_2 <= angle_throt->pitch)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }
  if (angle_throt->throttle < tobas::kMinThrot || tobas::kMaxThrot < angle_throt->throttle)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target throttle is invalid.");
    return;
  }

  // 外側の制御を止める
  pvay_cmd_ = nullptr;

  // コマンドを更新
  if (angle_cmd_ == nullptr)
    angle_cmd_ = std::make_shared<AngleThrust>();
  angle_cmd_->rpy.roll = angle_throt->roll;
  angle_cmd_->rpy.pitch = angle_throt->pitch;
  angle_cmd_->rpy.yaw = angle_throt->yaw;
  angle_cmd_->thrust = z_rotors_.maxThrustSum() * angle_throt->throttle;
}

void ControllerNode::rateThrustCmdCb(const tobas_command_msgs::msg::RateThrottle::ConstSharedPtr& rate_throt)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(rate_throt->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // Check command range
  if (rate_throt->throttle < tobas::kMinThrot || tobas::kMaxThrot < rate_throt->throttle)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target throttle is invalid.");
    return;
  }

  // 外側の制御を止める
  pvay_cmd_ = nullptr;
  angle_cmd_ = nullptr;

  // コマンドを更新
  if (rate_cmd_ == nullptr)
    rate_cmd_ = std::make_shared<RateThrust>();
  rate_cmd_->drpy.roll = rate_throt->droll;
  rate_cmd_->drpy.pitch = rate_throt->dpitch;
  rate_cmd_->drpy.yaw = rate_throt->dyaw;
  rate_cmd_->thrust = z_rotors_.maxThrustSum() * rate_throt->throttle;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
