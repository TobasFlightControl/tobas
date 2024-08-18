#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl/treejntparser.hpp>
#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_pose_pid/orientation_pid.hpp>

#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/PoseTwistAccelCommand.hpp>
#include <tobas_kdl_msgs/Tree.hpp>
#include <tobas_drone_msgs/Drone.hpp>
#include <tobas_debug_msgs/NonPlanarControllerFeedback.hpp>

#include "../include/tobas_np_pid/mixer.hpp"

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

  kdl::TreeJointStateConverter js_converter_;

  // Controllers
  tobas::PositionPid pos_pid_;
  tobas::OrientationPid ori_pid_;
  tobas::NonPlanarMixer mixer_;

  // Dynamic parameters
  tobas::PositionPidConfig pos_cfg_;
  tobas::OrientationPidConfig ori_cfg_;
  tobas::NonPlanarMixerConfig mixer_cfg_;

  // Mutable variables
  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  sensor_msgs::msg::JointState::ConstSharedPtr js_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;
  tobas_msgs::PoseTwistAccelCommand::SharedPtr cmd_;
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  PublisherPtr<tobas_msgs::msg::RotorSpeeds> rot_speeds_pub_;
  PublisherPtr<tobas_debug_msgs::NonPlanarControllerFeedback> feedback_pub_;

  // Subscribers
  SubscriberPtr<tobas::Drone> drone_sub_;
  SubscriberPtr<kdl::Tree> tree_sub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  SubscriberPtr<sensor_msgs::msg::JointState> js_sub_;
  SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  SubscriberPtr<tobas_msgs::PoseTwistAccelCommand> cmd_sub_;

  void initialize();
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
  bool mixerLinearWeightCb(const long& p);
  bool mixerAngularWeightCb(const long& p);
  bool mixerThrustWeightLog10Cb(const long& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& js);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void commandCb(const tobas_msgs::PoseTwistAccelCommand::ConstSharedPtr& cmd);
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::kControllerNode, options), js_converter_(tree_), mixer_(drone_, tree_)
{
  // Register dynamic parameters
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 1., 0.7, 1.0);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0.1, 0.1, 10.);
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0.1, 0.1, 10.);
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFrequencyCb, this, 10., 1., 20.);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0.1, 0.1, 40.);
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFrequencyCb, this, 5., 0.1, 10.);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0.1, 0.1, 20.);
  addDynamicDoubleParam("max_horizontal_accel", &self::maxHorizontalAccelCb, this, 10., 1., 15.);
  addDynamicDoubleParam("max_vertical_accel", &self::maxVerticalAccelCb, this, 8., 1., 10.);
  addDynamicIntParam("mixer_linear_weight", &self::mixerLinearWeightCb, this, 50, 1, 100);
  addDynamicIntParam("mixer_angular_weight", &self::mixerAngularWeightCb, this, 50, 1, 100);
  addDynamicIntParam("mixer_thrust_weight_log10", &self::mixerThrustWeightLog10Cb, this, -6, -9, 0);
  publishDynamicParameterDescriptions();

  // Register publishers
  rot_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::NonPlanarControllerFeedback>(tobas::kControllerFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryLpfTopic, &self::batteryCb, this);
  if (drone_.isTransformable())
    js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::jointStateCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  cmd_sub_ = createSubscriber(tobas::kPoseTwistAccelCmdTopic, &self::commandCb, this);
}

void ControllerNode::initialize()
{
  js_converter_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

  pos_pid_.configure(pos_cfg_);
  ori_pid_.configure(ori_cfg_);
  mixer_.configure(mixer_cfg_);

  is_initialized_ = true;
}

bool ControllerNode::isReadyToControl()
{
  if (!drone_received_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kDroneTopic);
    return false;
  }

  if (!tree_received_)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kRobotDescriptionTopic);
    return false;
  }

  if (odom_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kOdometryTopic);
    return false;
  }

  if (odom_->status != tobas_msgs::msg::Odometry::NO_ERROR)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "There is a problem with the state estimation.");
    return false;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kBatteryLpfTopic);
    return false;
  }

  if (drone_.isTransformable() && js_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kJointStatesTopic);
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

bool ControllerNode::horizontalNaturalFrequencyCb(const double& p)
{
  pos_cfg_.hor_natural_freq = p;
  if (is_initialized_)
    pos_pid_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::horizontalDampingRatioCb(const double& p)
{
  pos_cfg_.hor_damp_ratio = p;
  if (is_initialized_)
    pos_pid_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::horizontalIGainCb(const double& p)
{
  pos_cfg_.hor_ki = p;
  if (is_initialized_)
    pos_pid_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::verticalNaturalFrequencyCb(const double& p)
{
  pos_cfg_.ver_natural_freq = p;
  if (is_initialized_)
    pos_pid_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::verticalDampingRatioCb(const double& p)
{
  pos_cfg_.ver_damp_ratio = p;
  if (is_initialized_)
    pos_pid_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::verticalIGainCb(const double& p)
{
  pos_cfg_.ver_ki = p;
  if (is_initialized_)
    pos_pid_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::attitudeNaturalFrequencyCb(const double& p)
{
  ori_cfg_.atti_natural_freq = p;
  if (is_initialized_)
    ori_pid_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::attitudeDampingRatioCb(const double& p)
{
  ori_cfg_.atti_damp_ratio = p;
  if (is_initialized_)
    ori_pid_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::attitudeIGainCb(const double& p)
{
  ori_cfg_.atti_ki = p;
  if (is_initialized_)
    ori_pid_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::headingNaturalFrequencyCb(const double& p)
{
  ori_cfg_.head_natural_freq = p;
  if (is_initialized_)
    ori_pid_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::headingDampingRatioCb(const double& p)
{
  ori_cfg_.head_damp_ratio = p;
  if (is_initialized_)
    ori_pid_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::headingIGainCb(const double& p)
{
  ori_cfg_.head_ki = p;
  if (is_initialized_)
    ori_pid_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::maxHorizontalAccelCb(const double& p)
{
  pos_cfg_.max_hor_acc = p;
  if (is_initialized_)
    pos_pid_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::maxVerticalAccelCb(const double& p)
{
  pos_cfg_.max_ver_acc = p;
  if (is_initialized_)
    pos_pid_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::mixerLinearWeightCb(const long& p)
{
  mixer_cfg_.linear_weight = p;
  if (is_initialized_)
    mixer_.configure(mixer_cfg_);
  return true;
}

bool ControllerNode::mixerAngularWeightCb(const long& p)
{
  mixer_cfg_.angular_weight = p;
  if (is_initialized_)
    mixer_.configure(mixer_cfg_);
  return true;
}

bool ControllerNode::mixerThrustWeightLog10Cb(const long& p)
{
  mixer_cfg_.thrust_weight_log10 = p;
  if (is_initialized_)
    mixer_.configure(mixer_cfg_);
  return true;
}

void ControllerNode::droneCb(const tobas::Drone::ConstSharedPtr& drone)
{
  drone_ = *drone;
  drone_received_ = true;

  if (tree_received_)
    initialize();
}

void ControllerNode::treeCb(const kdl::Tree::ConstSharedPtr& tree)
{
  tree_ = *tree;
  tree_received_ = true;

  if (drone_received_)
    initialize();
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

  // コマンドが来ていなければスキップ
  if (cmd_ == nullptr)
    return;

  // 可動関節の角度を更新
  if (drone_.isTransformable() && js_converter_.jointStateToJntArrayPos(*js_) < 0)
    TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());

  // 位置制御器
  const auto cur_vel_W = odom->frame.M * odom->twist.vel;  // 世界座標系から見た現在の速度
  const kdl::Vector tar_acc_fb(pos_pid_.update(odom->frame.p.data, cur_vel_W.data, cmd_->pos.data, cmd_->vel.data, dt));
  const auto tar_acc_W = cmd_->acc + tar_acc_fb;

  // 姿勢制御器
  const auto tar_dgyro_fb = ori_pid_.update(kdl::Euler(odom->frame.M), odom->twist.rot, cmd_->rpy, cmd_->gyro, dt);
  const auto tar_dgyro_B = cmd_->dgyro + tar_dgyro_fb;

  // ミキサーで6軸加速度をプロペラの推力に変換
  const auto thrusts = mixer_.solve(
    battery_->voltage, js_converter_.getPositionsKDL(), odom->frame.M, odom->twist.rot, tar_acc_W, tar_dgyro_B);

  // 目標回転数を発行
  auto tar_rot_speeds = std::make_unique<tobas_msgs::msg::RotorSpeeds>();
  tar_rot_speeds->header.stamp = odom->header.stamp;
  tar_rot_speeds->speeds.resize(drone_.numRotors(), 0.);
  for (size_t rotor_idx = 0; rotor_idx < static_cast<size_t>(thrusts.rows()); ++rotor_idx)
  {
    const auto thrust = max(0., thrusts(rotor_idx));
    tar_rot_speeds->speeds[rotor_idx] = drone_.rotSpeedFromThrust(rotor_idx, thrust);
  }
  rot_speeds_pub_->publish(move(tar_rot_speeds));

  // フィードバックを発行
  // 目標位置速度はコマンドそのままだが，発行されていない間も安定して描画するためにメッセージに含めている
  auto feedback = std::make_unique<tobas_debug_msgs::NonPlanarControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;
  feedback->target_position = cmd_->pos;
  feedback->target_orientation = cmd_->rpy;
  feedback->target_twist_local.vel = odom->frame.M.inverse(cmd_->vel);
  feedback->target_twist_local.rot = cmd_->gyro;
  feedback->target_twist_global.vel = cmd_->vel;
  feedback->target_twist_global.rot = odom->frame.M * cmd_->gyro;
  feedback->target_accel_local.linear = odom->frame.M.inverse(tar_acc_W);
  feedback->target_accel_local.angular = tar_dgyro_B;
  feedback->target_accel_global.linear = tar_acc_W;
  feedback->target_accel_global.angular = odom->frame.M * tar_dgyro_B;
  feedback->position_integral_error.data = pos_pid_.integralError();
  feedback->orientation_integral_error = kdl::Euler(ori_pid_.integralError());
  feedback_pub_->publish(move(feedback));
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
  arming_ = arming;

  // Disarm時にコマンドをリセットする．でないと再度アームした時に前回のコマンドでモータが回り始めてしまう．
  if (!arming->data)
  {
    cmd_ = nullptr;
    TOBAS_INFO("Command is reset.");
  }
}

void ControllerNode::commandCb(const tobas_msgs::PoseTwistAccelCommand::ConstSharedPtr& cmd)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(cmd->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // コマンドを更新
  cmd_ = std::make_shared<tobas_msgs::PoseTwistAccelCommand>(*cmd);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::msg::FrameId::WORLD, odom_->frame.M, *cmd_))
  {
    TOBAS_ERROR("Failed to change command frame. Probably the frame id is invalid.");
    cmd_ = nullptr;
    return;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
