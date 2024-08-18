#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tobas_kdl/treejointstateconverter.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_pose_pid/orientation_pid.hpp>
#include <tobas_drone_tools/mr_accel_attitude_converter.hpp>
#include <tobas_drone_tools/mr_mixer.hpp>
#include <tobas_constants/constants.hpp>

#include <tobas_msgs/Odometry.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_msgs/PosVelAccYaw.hpp>
#include <tobas_msgs/RollPitchYawThrust.hpp>
#include <tobas_kdl_msgs/Tree.hpp>
#include <tobas_drone_msgs/Drone.hpp>
#include <tobas_debug_msgs/MultiRotorControllerFeedback.hpp>

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
  tobas::RotorAxisExtractor z_rotors_;

  // Static parameters
  bool do_thrust_correction_;

  // Controllers
  tobas::PositionPid pos_ctrl_;
  tobas::AccelAttitudeConverter acc_ctrl_;
  tobas::OrientationPid ori_ctrl_;
  tobas::Mixer mixer_;

  // Dynamic parameters
  tobas::PositionPidConfig pos_cfg_;
  tobas::AccelAttitudeConverterConfig acc_cfg_;
  tobas::OrientationPidConfig ori_cfg_;

  // Mutable variables
  bool is_initialized_ = false;
  bool drone_received_ = false;
  bool tree_received_ = false;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  sensor_msgs::msg::JointState::ConstSharedPtr js_;
  std_msgs::msg::Float64::ConstSharedPtr thrust_corr_factor_;
  std_msgs::msg::Bool::ConstSharedPtr arming_;
  tobas_msgs::PosVelAccYaw::SharedPtr tar_pvay_W_;      // PosVelYawの目標値 (世界座標系)
  tobas_msgs::RollPitchYawThrust::SharedPtr tar_rpyt_;  // RollPitchYawThrustの目標値
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  PublisherPtr<tobas_msgs::msg::RotorSpeeds> rot_speeds_pub_;
  PublisherPtr<tobas_debug_msgs::MultiRotorControllerFeedback> feedback_pub_;

  // Subscribers
  SubscriberPtr<tobas::Drone> drone_sub_;
  SubscriberPtr<kdl::Tree> tree_sub_;
  SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  SubscriberPtr<sensor_msgs::msg::JointState> js_sub_;
  SubscriberPtr<std_msgs::msg::Float64> thrust_factor_sub_;
  SubscriberPtr<std_msgs::msg::Bool> arming_sub_;
  SubscriberPtr<tobas_msgs::PosVelAccYaw> pvay_sub_;
  SubscriberPtr<tobas_msgs::RollPitchYawThrust> rpyt_sub_;

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
  bool maxAttitudeCb(const double& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& js);
  void thrustFactorCb(const std_msgs::msg::Float64::ConstSharedPtr& msg);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
  void posVelAccYawCb(const tobas_msgs::PosVelAccYaw::ConstSharedPtr& pvay);
  void rpyThrustCb(const tobas_msgs::RollPitchYawThrust::ConstSharedPtr& rpyt);
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::kControllerNode, options),
    js_converter_(tree_),
    z_rotors_(drone_, tobas::Z_POSITIVE),
    acc_ctrl_(drone_, tree_),
    mixer_(drone_, tree_)
{
  // TODO: 動的パラメータで調節できるように
  // TODO: そもそも風の補償方法を見直すべき
  acc_cfg_.h_force_comp_rate = 0;

  // Get static parameters
  do_thrust_correction_ = getBoolParam("do_thrust_correction", false);

  // Register dynamic parameters
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFrequencyCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 1., 0.7, 1.0);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0.1, 0.1, 10.);
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0.1, 0.1, 10.);
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFrequencyCb, this, 10., 1., 20.);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0.1, 0.1, 40.);
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFrequencyCb, this, 3., 0.1, 5.);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 1., 0.7, 1.);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0.1, 0.1, 20.);
  addDynamicDoubleParam("max_horizontal_accel", &self::maxHorizontalAccelCb, this, 8., 1., 10.);
  addDynamicDoubleParam("max_vertical_accel", &self::maxVerticalAccelCb, this, 4., 1., 10.);
  addDynamicDoubleParam("max_attitude", &self::maxAttitudeCb, this, M_PI / 3, 0., M_PI_2 - 1e-3);
  publishDynamicParameterDescriptions();

  // Register publishers
  rot_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::MultiRotorControllerFeedback>(tobas::kControllerFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryLpfTopic, &self::batteryCb, this);
  if (drone_.isTransformable())
    js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::jointStateCb, this);
  if (do_thrust_correction_)
    thrust_factor_sub_ = createSubscriber(tobas::kThrustCorrectionFactorTopic, &self::thrustFactorCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  pvay_sub_ = createSubscriber(tobas::kPosVelAccYawCmdTopic, &self::posVelAccYawCb, this);
  rpyt_sub_ = createSubscriber(tobas::kRpyThrustCmdTopic, &self::rpyThrustCb, this);
}

void ControllerNode::initialize()
{
  z_rotors_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
  acc_ctrl_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

  pos_ctrl_.configure(pos_cfg_);
  acc_ctrl_.configure(acc_cfg_);
  ori_ctrl_.configure(ori_cfg_);

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

  if (do_thrust_correction_ && thrust_corr_factor_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kThrustCorrectionFactorTopic);
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
    pos_ctrl_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::horizontalDampingRatioCb(const double& p)
{
  pos_cfg_.hor_damp_ratio = p;
  if (is_initialized_)
    pos_ctrl_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::horizontalIGainCb(const double& p)
{
  pos_cfg_.hor_ki = p;
  if (is_initialized_)
    pos_ctrl_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::verticalNaturalFrequencyCb(const double& p)
{
  pos_cfg_.ver_natural_freq = p;
  if (is_initialized_)
    pos_ctrl_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::verticalDampingRatioCb(const double& p)
{
  pos_cfg_.ver_damp_ratio = p;
  if (is_initialized_)
    pos_ctrl_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::verticalIGainCb(const double& p)
{
  pos_cfg_.ver_ki = p;
  if (is_initialized_)
    pos_ctrl_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::attitudeNaturalFrequencyCb(const double& p)
{
  ori_cfg_.atti_natural_freq = p;
  if (is_initialized_)
    ori_ctrl_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::attitudeDampingRatioCb(const double& p)
{
  ori_cfg_.atti_damp_ratio = p;
  if (is_initialized_)
    ori_ctrl_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::attitudeIGainCb(const double& p)
{
  ori_cfg_.atti_ki = p;
  if (is_initialized_)
    ori_ctrl_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::headingNaturalFrequencyCb(const double& p)
{
  ori_cfg_.head_natural_freq = p;
  if (is_initialized_)
    ori_ctrl_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::headingDampingRatioCb(const double& p)
{
  ori_cfg_.head_damp_ratio = p;
  if (is_initialized_)
    ori_ctrl_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::headingIGainCb(const double& p)
{
  ori_cfg_.head_ki = p;
  if (is_initialized_)
    ori_ctrl_.configure(ori_cfg_);
  return true;
}

bool ControllerNode::maxHorizontalAccelCb(const double& p)
{
  pos_cfg_.max_hor_acc = p;
  if (is_initialized_)
    pos_ctrl_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::maxVerticalAccelCb(const double& p)
{
  pos_cfg_.max_ver_acc = p;
  if (is_initialized_)
    pos_ctrl_.configure(pos_cfg_);
  return true;
}

bool ControllerNode::maxAttitudeCb(const double& p)
{
  acc_cfg_.max_attitude = p;
  if (is_initialized_)
    acc_ctrl_.configure(acc_cfg_);
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

  // Create a feedback message
  auto feedback = std::make_unique<tobas_debug_msgs::MultiRotorControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;

  // Translation Controller
  if (tar_pvay_W_ != nullptr)
  {
    if (tar_rpyt_ == nullptr)
      tar_rpyt_ = std::make_shared<tobas_msgs::RollPitchYawThrust>();

    // 世界座標系から見た現在の速度を計算
    const auto cur_vel_W = odom->frame.M * odom->twist.vel;

    // 目標加速度を計算
    const kdl::Vector tar_acc_fb(
      pos_ctrl_.update(odom->frame.p.data, cur_vel_W.data, tar_pvay_W_->pos.data, tar_pvay_W_->vel.data, dt));
    const auto tar_acc = tar_pvay_W_->acc + tar_acc_fb;

    // 推力和と目標姿勢を計算
    acc_ctrl_.update(odom->frame.M, tar_acc, tar_rpyt_->thrust, tar_rpyt_->rpy.roll, tar_rpyt_->rpy.pitch);

    // コマンドレベルとヨー角は加速度指令をそのまま流す
    tar_rpyt_->level = tar_pvay_W_->level;
    tar_rpyt_->rpy.yaw = tar_pvay_W_->yaw;

    // Fill feedback
    feedback->target_position = tar_pvay_W_->pos;
    feedback->target_velocity_global = tar_pvay_W_->vel;
    feedback->target_velocity_local = odom->frame.M.inverse(tar_pvay_W_->vel);
    feedback->target_accel_global = tar_acc;
    feedback->target_accel_local = odom->frame.M.inverse(tar_acc);
    feedback->position_integral_error.data = pos_ctrl_.integralError();
  }

  // Rotation Controller
  if (tar_rpyt_ != nullptr)
  {
    // 可動関節角を更新
    if (drone_.isTransformable() && js_converter_.jointStateToJntArrayPos(*js_) < 0)
      TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());

    // 目標角加速度を計算
    const auto tar_dgyro =
      ori_ctrl_.update(kdl::Euler(odom->frame.M), odom->twist.rot, tar_rpyt_->rpy, kdl::Vector::Zero(), dt);

    // プロペラの推力を計算
    // TODO: H-momentを考慮
    const VectorXd thrusts = mixer_.solve(
      dt, battery_->voltage, js_converter_.getPositionsKDL(), odom->twist.rot.data, Vector3d::Zero(), tar_dgyro.data,
      tar_rpyt_->thrust);

    // 目標回転数を発行
    auto tar_rot_speeds = std::make_unique<tobas_msgs::msg::RotorSpeeds>();
    tar_rot_speeds->header.stamp = odom->header.stamp;
    tar_rot_speeds->speeds.resize(drone_.numRotors(), 0.);
    for (size_t i = 0; i < static_cast<size_t>(thrusts.rows()); ++i)
    {
      auto thrust = max(0., thrusts(i));
      if (do_thrust_correction_ && thrust_corr_factor_ != nullptr)  // 推力補正
        thrust *= thrust_corr_factor_->data;
      tar_rot_speeds->speeds[z_rotors_.rotorIdx(i)] = z_rotors_.rotSpeedFromThrust(i, thrust);
    }
    rot_speeds_pub_->publish(move(tar_rot_speeds));

    // フィードバックを発行
    feedback->target_orientation = tar_rpyt_->rpy;
    feedback->target_thrust = tar_rpyt_->thrust;
    feedback_pub_->publish(move(feedback));
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

void ControllerNode::thrustFactorCb(const std_msgs::msg::Float64::ConstSharedPtr& msg)
{
  thrust_corr_factor_ = msg;
}

void ControllerNode::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
{
  arming_ = arming;

  // Disarm時にコマンドをリセットする．でないと再度アームした時に前回のコマンドでモータが回り始めてしまう．
  if (!arming->data)
  {
    tar_pvay_W_ = nullptr;
    tar_rpyt_ = nullptr;
    TOBAS_INFO("Command is reset.");
  }
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

void ControllerNode::rpyThrustCb(const tobas_msgs::RollPitchYawThrust::ConstSharedPtr& rpyt)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(rpyt->level.data, get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // 外側の制御を止める
  tar_pvay_W_ = nullptr;

  // コマンドを更新
  tar_rpyt_ = std::make_shared<tobas_msgs::RollPitchYawThrust>(*rpyt);
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
