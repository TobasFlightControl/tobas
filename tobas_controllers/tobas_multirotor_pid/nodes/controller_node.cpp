#include <ranges>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_pose_pid/euler_pid.hpp>
#include <tobas_drone_tools/mr_accel_attitude_converter.hpp>
#include <tobas_drone_tools/mr_mixer.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs_adapter/pos_vel_acc_yaw.hpp>
#include <tobas_msgs_adapter/roll_pitch_yaw_throttle.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_debug_msgs_adapter/multi_rotor_controller_feedback.hpp>

using namespace std;
using namespace Eigen;

struct RollPitchYawThrust
{
  tobas_msgs::msg::CommandLevel level;
  kdl::Euler rpy;
  double thrust;
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

  // Controllers
  tobas::PositionPID pos_pid_;
  tobas::AccelAttitudeConverter acc_atti_conv_;
  tobas::EulerPID rot_pid_;  // 平面配置MRだとなぜか角軸ベクトルよりオイラー角で姿勢誤差を計算した方が制御が安定する
  tobas::MultiRotorMixer mixer_;

  // Mutable variables
  bool drone_received_ = false;
  bool tree_received_ = false;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  sensor_msgs::msg::JointState::ConstSharedPtr js_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;
  tobas_msgs::PosVelAccYaw::SharedPtr tar_pvay_W_;  // PosVelYawの目標値 (世界座標系)
  shared_ptr<RollPitchYawThrust> tar_rpyt_;         // RollPitchYawThrustの目標値
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::MultiRotorControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<sensor_msgs::msg::JointState> js_sub_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::PosVelAccYaw> pvay_sub_;
  ros2::SubscriberPtr<tobas_msgs::RollPitchYawThrottle> rpyt_sub_;

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
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& js);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void posVelAccYawCb(const tobas_msgs::PosVelAccYaw::ConstSharedPtr& pvay);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrottle::ConstSharedPtr& rpy_throttle);
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::kControllerNode, options),
    js_converter_(tree_),
    z_rotors_(drone_, tobas::Z_POSITIVE),
    acc_atti_conv_(tree_),
    mixer_(drone_, tree_)
{
  // Register dynamic parameters
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFrequencyCb, this, 1., 0.1, 5.);
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFrequencyCb, this, 10., 1., 50.);
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFrequencyCb, this, 5., 0.1, 25.);
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0.1, 0.1, 10.);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0.1, 0.1, 10.);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0.1, 0.1, 40.);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0.1, 0.1, 20.);
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
  battery_sub_ = createSubscriber(tobas::kBatteryTopic, &self::batteryCb, this);
  js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::jointStateCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  pvay_sub_ = createSubscriber(tobas::kPosVelAccYawCmdTopic, &self::posVelAccYawCb, this);
  rpyt_sub_ = createSubscriber(tobas::kRPYThrotCmdTopic, &self::rpyThrustCb, this);
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

  if (odom_->status != tobas_msgs::msg::Odometry::NO_ERROR)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "There is a problem with the state estimation.");
    return false;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kBatteryTopic, "\".");
    return false;
  }

  if (drone_.isTransformable() && js_ == nullptr)
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
  return rot_pid_.setNaturalFreq(0, p) && rot_pid_.setNaturalFreq(1, p);
}

bool ControllerNode::attitudeDampingRatioCb(const double& p)
{
  return rot_pid_.setDampingRatio(0, p) && rot_pid_.setDampingRatio(1, p);
}

bool ControllerNode::attitudeIGainCb(const double& p)
{
  return rot_pid_.setIntegralGain(0, p) && rot_pid_.setIntegralGain(1, p);
}

bool ControllerNode::headingNaturalFrequencyCb(const double& p)
{
  return rot_pid_.setNaturalFreq(2, p);
}

bool ControllerNode::headingDampingRatioCb(const double& p)
{
  return rot_pid_.setDampingRatio(2, p);
}

bool ControllerNode::headingIGainCb(const double& p)
{
  return rot_pid_.setIntegralGain(2, p);
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

  if (!isReadyToControl())
    return;

  // Create a feedback message
  auto feedback_msg = std::make_unique<tobas_debug_msgs::MultiRotorControllerFeedback>();
  feedback_msg->header.stamp = odom->header.stamp;

  // Translation Controller
  if (tar_pvay_W_ != nullptr)
  {
    if (tar_rpyt_ == nullptr)
      tar_rpyt_ = std::make_shared<RollPitchYawThrust>();

    // 世界座標系から見た現在の位置速度
    const auto& cur_pos_W = odom->frame.p;
    const auto cur_vel_W = odom->frame.M * odom->twist.vel;

    // 目標加速度を計算
    const auto tar_acc_fb = pos_pid_.update(cur_pos_W, cur_vel_W, tar_pvay_W_->pos, tar_pvay_W_->vel, dt);
    const auto tar_acc = tar_pvay_W_->acc + tar_acc_fb;

    // 推力和と目標姿勢を計算
    acc_atti_conv_.update(odom->frame.M, tar_acc, tar_rpyt_->thrust, tar_rpyt_->rpy.roll, tar_rpyt_->rpy.pitch);

    // コマンドレベルとヨー角は加速度指令をそのまま流す
    tar_rpyt_->level = tar_pvay_W_->level;
    tar_rpyt_->rpy.yaw = tar_pvay_W_->yaw;

    // Fill feedback
    feedback_msg->target_position = tar_pvay_W_->pos;
    feedback_msg->target_velocity_global = tar_pvay_W_->vel;
    feedback_msg->target_velocity_local = odom->frame.M.inverse(tar_pvay_W_->vel);
    feedback_msg->target_accel_global = tar_acc;
    feedback_msg->target_accel_local = odom->frame.M.inverse(tar_acc);
    feedback_msg->position_integral_error = pos_pid_.integralError();
  }

  // Rotation Controller
  if (tar_rpyt_ != nullptr)
  {
    // 可動関節角を更新
    if (drone_.isTransformable() && js_converter_.jointStateToJntArrayPos(*js_) < 0)
      TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());

    // 目標角加速度を計算
    const auto tar_dgyro =
      rot_pid_.update(kdl::Euler(odom->frame.M), odom->twist.rot, tar_rpyt_->rpy, kdl::Vector::Zero(), dt);

    // プロペラの推力を計算
    // TODO: H-momentを考慮
    if (!mixer_.solve(
          battery_->voltage, js_converter_.getPositionsKDL(), odom->twist.rot, tar_dgyro, tar_rpyt_->thrust))
    {
      TOBAS_FATAL("Failed to solve mixing equation.");
      return;
    }
    const auto& thrusts = mixer_.getThrusts();

    // 目標回転数を発行
    auto thrusts_msg = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
    thrusts_msg->header.stamp = odom->header.stamp;
    for (const auto& [idx, rotor_it] : views::enumerate(drone_.rotors))
    {
      thrusts_msg->thrusts.emplace_back();
      thrusts_msg->thrusts.back().channel = rotor_it.first;
      thrusts_msg->thrusts.back().thrust = thrusts(idx);
    }
    tar_thrusts_pub_->publish(move(thrusts_msg));

    // フィードバックを発行
    feedback_msg->target_orientation = tar_rpyt_->rpy;
    feedback_msg->target_thrust = tar_rpyt_->thrust;
    feedback_pub_->publish(move(feedback_msg));
  }
}

void ControllerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void ControllerNode::jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& js)
{
  if (js->name.size() != js->position.size())
  {
    TOBAS_ERROR("The size of joint name and position is different.");
    return;
  }

  js_ = js;
}

void ControllerNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  // Disarm時にコマンドをリセットする．でないと再度アームした時に前回のコマンドでモータが回り始めてしまう．
  if (arming_ != nullptr && arming_->data && !arming->data)
  {
    tar_pvay_W_ = nullptr;
    tar_rpyt_ = nullptr;
    TOBAS_INFO("Command is reset.");
  }

  arming_ = arming;
}

void ControllerNode::posVelAccYawCb(const tobas_msgs::PosVelAccYaw::ConstSharedPtr& pvay)
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
  tar_pvay_W_ = std::make_shared<tobas_msgs::PosVelAccYaw>(*pvay);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::msg::FrameId::WORLD, odom_->frame.M, *tar_pvay_W_))
  {
    TOBAS_ERROR("Failed to change command frame. Probably the frame ID is invalid.");
    tar_pvay_W_ = nullptr;
    return;
  }
}

void ControllerNode::rpyThrustCb(const tobas_msgs::RollPitchYawThrottle::ConstSharedPtr& rpy_throttle)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(rpy_throttle->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // Check command range
  if (rpy_throttle->rpy.roll <= -M_PI_2 || M_PI_2 <= rpy_throttle->rpy.roll)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target roll is invalid.");
    return;
  }
  if (rpy_throttle->rpy.pitch <= -M_PI_2 || M_PI_2 <= rpy_throttle->rpy.pitch)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target pitch is invalid.");
    return;
  }
  if (rpy_throttle->throttle < tobas::kMinThrot || tobas::kMaxThrot < rpy_throttle->throttle)
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "Target throttle is invalid.");
    return;
  }

  // 外側の制御を止める
  tar_pvay_W_ = nullptr;

  // コマンドを更新
  if (tar_rpyt_ == nullptr)
    tar_rpyt_ = std::make_shared<RollPitchYawThrust>();
  tar_rpyt_->level = rpy_throttle->level;
  tar_rpyt_->rpy = rpy_throttle->rpy;
  tar_rpyt_->thrust = z_rotors_.thrustSum(battery_->voltage, rpy_throttle->throttle);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
