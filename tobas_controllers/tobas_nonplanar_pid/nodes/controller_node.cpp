#include <ranges>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_node/node.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_tools/tree_joint_state_converter.hpp>
#include <tobas_tools/command_level_handler.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_tools/np_mixer_qp.hpp>
#include <tobas_pose_pid/position_pid.hpp>
#include <tobas_pose_pid/angle_axis_pid.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_thrust_array.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs_adapter/odometry.hpp>
#include <tobas_command_msgs_adapter/pose_twist_accel.hpp>
#include <tobas_kdl_msgs_adapter/tree.hpp>
#include <tobas_kdl_msgs_adapter/wrench_stamped.hpp>
#include <tobas_drone_msgs_adapter/drone.hpp>
#include <tobas_debug_msgs_adapter/non_planar_controller_feedback.hpp>

using namespace std;
using namespace Eigen;

class ControllerNode : public tobas::BaseNode
{
  using self = ControllerNode;
  using super = tobas::BaseNode;

  static constexpr long kMaxWeight = 100;

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
  tobas::AngleAxisPID rot_pid_;  // ジンバルロック回避のため姿勢のPIDを角軸ベクトルで計算
  tobas::NonPlanarMixer_QP mixer_;

  // Mutable variables
  bool drone_received_ = false;
  bool tree_received_ = false;
  bool js_received_ = false;
  tobas_msgs::Odometry::ConstSharedPtr odom_;
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  tobas_kdl_msgs::WrenchStamped::ConstSharedPtr dist_force_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_command_msgs::PoseTwistAccel::SharedPtr cmd_;
  tobas::CommandLevelHandler cmd_level_handler_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorThrustArray> tar_thrusts_pub_;
  ros2::PublisherPtr<tobas_debug_msgs::NonPlanarControllerFeedback> feedback_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas::Drone> drone_sub_;
  ros2::SubscriberPtr<kdl::Tree> tree_sub_;
  ros2::SubscriberPtr<tobas_msgs::Odometry> odom_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_sub_;
  ros2::SubscriberPtr<tobas_kdl_msgs::WrenchStamped> dist_force_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::JointStateArray> js_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_sub_;
  ros2::SubscriberPtr<tobas_command_msgs::PoseTwistAccel> cmd_sub_;

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
  bool mixerLinearWeightCb(const long& p);
  bool mixerAngularWeightCb(const long& p);
  bool mixerThrustWeightLog10Cb(const long& p);

  void droneCb(const tobas::Drone::ConstSharedPtr& drone);
  void treeCb(const kdl::Tree::ConstSharedPtr& tree);
  void odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom);
  void batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void disturbanceForceCb(const tobas_kdl_msgs::WrenchStamped::ConstSharedPtr& dist_force);
  void jointStateCb(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& js);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness);
  void commandCb(const tobas_command_msgs::PoseTwistAccel::ConstSharedPtr& cmd);
};

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
  : super(tobas::node::kController, options), js_converter_(tree_), mixer_(drone_, tree_)
{
  // Get static parameters
  do_dist_comp_trans_ = getBoolParam("do_disturbance_compensation_translation", true);
  do_dist_comp_rot_ = getBoolParam("do_disturbance_compensation_rotation", false);

  // Register dynamic parameters
  addDynamicDoubleParam("horizontal_natural_frequency", &self::horizontalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("vertical_natural_frequency", &self::verticalNaturalFrequencyCb, this, 2., 0.1, 5.);
  addDynamicDoubleParam("attitude_natural_frequency", &self::attitudeNaturalFrequencyCb, this, 20., 1., 50.);
  addDynamicDoubleParam("heading_natural_frequency", &self::headingNaturalFrequencyCb, this, 10., 0.1, 25.);
  addDynamicDoubleParam("horizontal_damping_ratio", &self::horizontalDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("vertical_damping_ratio", &self::verticalDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("attitude_damping_ratio", &self::attitudeDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("heading_damping_ratio", &self::headingDampingRatioCb, this, 1., 0.1, 3.);
  addDynamicDoubleParam("horizontal_i_gain", &self::horizontalIGainCb, this, 0., 0., 10.);
  addDynamicDoubleParam("vertical_i_gain", &self::verticalIGainCb, this, 0., 0., 10.);
  addDynamicDoubleParam("attitude_i_gain", &self::attitudeIGainCb, this, 0., 0., 40.);
  addDynamicDoubleParam("heading_i_gain", &self::headingIGainCb, this, 0., 0., 20.);
  addDynamicDoubleParam("max_horizontal_accel", &self::maxHorizontalAccelCb, this, 10., 1., 20.);
  addDynamicDoubleParam("max_vertical_accel", &self::maxVerticalAccelCb, this, 8., 1., 10.);
  addDynamicIntParam("mixer_linear_weight", &self::mixerLinearWeightCb, this, kMaxWeight / 2, 1, kMaxWeight);
  addDynamicIntParam("mixer_angular_weight", &self::mixerAngularWeightCb, this, kMaxWeight / 2, 1, kMaxWeight);
  addDynamicIntParam("mixer_thrust_weight_log10", &self::mixerThrustWeightLog10Cb, this, -6, -9, 0);

  // Register publishers
  tar_thrusts_pub_ = createPublisher<tobas_msgs::msg::RotorThrustArray>(tobas::kRotorThrustsCmdTopic);
  feedback_pub_ = createPublisher<tobas_debug_msgs::NonPlanarControllerFeedback>(tobas::kNPCtrlFeedbackTopic);

  // Register subscribers
  drone_sub_ = createSubscriber(tobas::kDroneTopic, &self::droneCb, this, true, true);
  tree_sub_ = createSubscriber(tobas::kKDLTreeTopic, &self::treeCb, this, true, true);
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryTopic, &self::batteryCb, this);
  dist_force_sub_ = createSubscriber(tobas::kDisturbanceForceTopic, &self::disturbanceForceCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);
  rotor_liveliness_sub_ = createSubscriber(tobas::kRotorLivelinessTopic, &self::rotorLivelinessCb, this);
  cmd_sub_ = createSubscriber(tobas::kPoseTwistAccelCmdTopic, &self::commandCb, this);
}

bool ControllerNode::updateInternalDataStructures()
{
  if (!js_converter_.updateInternalDataStructures())
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
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", 12, tobas::kKDLTreeTopic, "\".");
    return false;
  }

  if (odom_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kOdometryTopic, "\".");
    return false;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for \"", tobas::kBatteryTopic, "\".");
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

bool ControllerNode::mixerLinearWeightCb(const long& p)
{
  return mixer_.setLinearWeight(static_cast<double>(p) / static_cast<double>(kMaxWeight));
}

bool ControllerNode::mixerAngularWeightCb(const long& p)
{
  return mixer_.setAngularWeight(static_cast<double>(p) / static_cast<double>(kMaxWeight));
}

bool ControllerNode::mixerThrustWeightLog10Cb(const long& p)
{
  return mixer_.setThrustWeight(exp10(p));
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

  // コマンドがなければスキップ
  if (cmd_ == nullptr)
    return;

  // 位置制御器
  const auto cur_vel_W = odom->frame.M * odom->twist.vel;  // 世界座標系から見た現在の速度
  const auto tar_acc_fb = pos_pid_.update(odom->frame.p, cur_vel_W, cmd_->pos, cmd_->vel, dt);
  const auto tar_acc_W = cmd_->acc + tar_acc_fb;

  // 姿勢制御器
  const auto tar_dgyro_fb = rot_pid_.update(odom->frame.M, odom->twist.rot, cmd_->rpy.toRotation(), cmd_->gyro, dt);
  const auto tar_dgyro_B = cmd_->dgyro + tar_dgyro_fb;

  // ミキサーで6軸加速度をプロペラの推力に変換
  const auto& dist_force_W = do_dist_comp_trans_ ? dist_force_->wrench.force : kdl::Vector::Zero();
  const auto& dist_torque_B = do_dist_comp_rot_ ? dist_force_->wrench.torque : kdl::Vector::Zero();
  if (!mixer_.solve(
        battery_->voltage, js_converter_.getPosition(), odom->frame.M, odom->twist.rot, tar_acc_W, tar_dgyro_B,
        dist_force_W, dist_torque_B))
  {
    TOBAS_FATAL("Failed to solve mixing equation.");
    return;
  }
  const auto& thrusts = mixer_.getThrusts();

  // 目標推力を発行
  auto tar_thrusts = std::make_unique<tobas_msgs::msg::RotorThrustArray>();
  tar_thrusts->header.stamp = odom->header.stamp;
  for (const auto& [idx, rotor_it] : views::enumerate(drone_.rotors))
  {
    tar_thrusts->thrusts.emplace_back();
    tar_thrusts->thrusts.back().channel = rotor_it.first;
    tar_thrusts->thrusts.back().thrust = thrusts(idx);
  }
  tar_thrusts_pub_->publish(move(tar_thrusts));

  // フィードバックを発行
  // 目標位置速度はコマンドそのままだが，発行されていない間も安定して描画するためにメッセージに含めている
  auto feedback = std::make_unique<tobas_debug_msgs::NonPlanarControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;
  feedback->target_position = cmd_->pos;
  feedback->target_orientation = cmd_->rpy;
  feedback->target_twist.vel = cmd_->vel;
  feedback->target_twist.rot = odom->frame.M * cmd_->gyro;
  feedback->target_accel.linear = tar_acc_W;
  feedback->target_accel.angular = odom->frame.M * tar_dgyro_B;
  feedback->position_integral_error = pos_pid_.integralError();
  feedback->orientation_integral_error = kdl::Euler(rot_pid_.integralError());
  feedback_pub_->publish(move(feedback));
}

void ControllerNode::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
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
    cmd_ = nullptr;
    TOBAS_INFO("Command is reset.");
  }

  arming_ = arming;
}

void ControllerNode::rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& rotor_liveliness)
{
  for (const auto& data : rotor_liveliness->data)
    if (!mixer_.setRotorLiveliness(data.channel, data.alive))
      TOBAS_ERROR("Failed to set the liveliness of rotor channel ", data.channel);
}

void ControllerNode::commandCb(const tobas_command_msgs::PoseTwistAccel::ConstSharedPtr& cmd)
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
  cmd_ = std::make_shared<tobas_command_msgs::PoseTwistAccel>(*cmd);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_command_msgs::msg::FrameId::WORLD, odom_->frame.M, *cmd_))
  {
    TOBAS_ERROR("Failed to change command frame. Probably the frame id is invalid.");
    cmd_ = nullptr;
    return;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ControllerNode)
