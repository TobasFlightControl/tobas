
#include <tobas_constants/constants.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>
#include <tobas_mr_pid/ControllerFeedback.h>

#include "../include/tobas_mr_pid/controller_ros.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_mr_pid
{
ControllerRos::ControllerRos(const rclcpp::NodeOptions& options)
  : super(node, pnh, name),
    js_converter_(tree_),
    z_rotors_(drone_, tobas::Z_POSITIVE),
    acc_ctrl_(drone_),
    mixer_(drone_),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(node_);

  z_rotors_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
  acc_ctrl_.updateInternalDataStructures();
  mixer_.updateInternalDataStructures();

  registerPublishers();
  registerSubscribers();

  server_.setCallback(std::bind(&self::dynamicReconfigureCb, this, _1, _2));
}

void ControllerRos::getRosParams()
{
  ros2::getParam(pnh_, "do_thrust_correction", do_thrust_correction_, kDefaultDoThrustCorrection);
}

void ControllerRos::registerPublishers()
{
  rot_speeds_pub_ = createPublisher<tobas_msgs::msg::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic);
  feedback_pub_ = createPublisher<tobas_mr_pid::ControllerFeedback>(tobas::kControllerFeedbackTopic);
}

void ControllerRos::registerSubscribers()
{
  odom_sub_ = createSubscriber(tobas::kOdometryTopic, &self::odomCb, this);
  battery_sub_ = createSubscriber(tobas::kBatteryLpfTopic, &self::batteryCb, this);
  if (drone_.isTransformable())
    js_sub_ = createSubscriber(tobas::kJointStatesTopic, &self::jointStateCb, this);
  if (do_thrust_correction_)
    thrust_corr_factor_sub_ =
      createSubscriber(tobas::kThrustCorrectionFactorTopic, &self::thrustCorrectionFactorCb, this);
  arming_sub_ = createSubscriber(tobas::kArmingTopic, &self::armingCb, this);

  pvay_sub_ = createSubscriber(tobas::kPosVelAccYawCmdTopic, &self::posVelAccYawCb, this);
  rpyt_sub_ = createSubscriber(tobas::kRpyThrustCmdTopic, &self::rpyThrustCb, this);
}

bool ControllerRos::isReadyToControl()
{
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

void ControllerRos::odomCb(const tobas_msgs::Odometry::ConstSharedPtr& odom)
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
  const auto feedback =std::make_unique<tobas_mr_pid::ControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;

  // Translation Controller
  if (tar_pvay_W_ != nullptr)
  {
    if (tar_rpyt_ == nullptr)
      tar_rpyt_ =std::make_unique<tobas_msgs::RollPitchYawThrust>();

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
    feedback->target_acceleration_global = tar_acc;
    feedback->target_acceleration_local = odom->frame.M.inverse(tar_acc);
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
    const auto tar_rot_speeds =std::make_unique<tobas_msgs::msg::RotorSpeeds>();
    tar_rot_speeds->header.stamp = odom->header.stamp;
    tar_rot_speeds->speeds.resize(drone_.rotors.size(), 0.);
    for (size_t i = 0; i < static_cast<size_t>(thrusts.rows()); ++i)
    {
      auto thrust = max(0., thrusts(i));
      if (do_thrust_correction_ && thrust_corr_factor_ != nullptr)  // 推力補正
        thrust *= thrust_corr_factor_->data;
      tar_rot_speeds->speeds[z_rotors_.rotorIdx(i)] = z_rotors_.rotSpeedFromThrust(i, thrust);
    }
    rot_speeds_pub_->publish(tar_rot_speeds);

    // フィードバックを発行
    feedback->target_orientation = tar_rpyt_->rpy;
    feedback->target_thrust = tar_rpyt_->thrust;
    feedback_pub_->publish(feedback);
  }
}

void ControllerRos::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void ControllerRos::jointStateCb(const sensor_msgs::msg::JointState::ConstSharedPtr& js)
{
  if (js->name.size() != js->position.size())
  {
    TOBAS_ERROR("The size of joint name and position is different.");
    return;
  }

  js_ = js;
}

void ControllerRos::thrustCorrectionFactorCb(const std_msgs::Float64::ConstSharedPtr& msg)
{
  thrust_corr_factor_ = msg;
}

void ControllerRos::armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming)
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

void ControllerRos::posVelAccYawCb(const tobas_msgs::PosVelAccYaw::ConstSharedPtr& pvay)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(pvay->level.data, node->get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // コマンドを更新
  tar_pvay_W_ =std::make_unique<tobas_msgs::PosVelAccYaw>(*pvay);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::msg::FrameId::WORLD, odom_->frame.M, *tar_pvay_W_))
  {
    TOBAS_ERROR("Failed to change command frame. Probably the frame ID is invalid.");
    tar_pvay_W_ = nullptr;
    return;
  }
}

void ControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrust::ConstSharedPtr& rpyt)
{
  if (!isReadyToControl())
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because the controller is not ready.");
    return;
  }

  if (!cmd_level_handler_.update(rpyt->level.data, node->get_clock()->now()))
  {
    TOBAS_WARN_THROTTLE(tobas::kIgnoreCmdMsgPeriod, "The command is ignored because of the its priority.");
    return;
  }

  // 外側の制御を止める
  tar_pvay_W_ = nullptr;

  // コマンドを更新
  tar_rpyt_ =std::make_unique<tobas_msgs::RollPitchYawThrust>(*rpyt);
}

void ControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  // 位置制御
  pos_cfg_.hor_natural_freq = cfg.horizontal_natural_frequency;
  pos_cfg_.hor_damp_ratio = cfg.horizontal_damping_ratio;
  pos_cfg_.hor_ki = cfg.horizontal_i_gain;
  pos_cfg_.ver_natural_freq = cfg.vertical_natural_frequency;
  pos_cfg_.ver_damp_ratio = cfg.vertical_damping_ratio;
  pos_cfg_.ver_ki = cfg.vertical_i_gain;
  pos_cfg_.max_hor_acc = cfg.max_horizontal_accel;
  pos_cfg_.max_ver_acc = cfg.max_vertical_accel;
  pos_ctrl_.configure(pos_cfg_);

  // 非線形変換
  acc_cfg_.max_attitude = cfg.max_attitude;
  acc_cfg_.h_force_comp_rate = 0;  // TODO
  acc_ctrl_.configure(acc_cfg_);

  // 姿勢制御器
  ori_cfg_.atti_natural_freq = cfg.attitude_natural_frequency;
  ori_cfg_.atti_damp_ratio = cfg.attitude_damping_ratio;
  ori_cfg_.atti_ki = cfg.attitude_i_gain;
  ori_cfg_.head_natural_freq = cfg.heading_natural_frequency;
  ori_cfg_.head_damp_ratio = cfg.heading_damping_ratio;
  ori_cfg_.head_ki = cfg.heading_i_gain;
  ori_ctrl_.configure(ori_cfg_);

  // TODO: Mixerの設定

  TOBAS_INFO("Dynamic parameters are updated.");
}
}  // namespace tobas_mr_pid
