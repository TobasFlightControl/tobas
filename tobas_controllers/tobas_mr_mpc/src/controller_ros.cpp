#include <tobas_ros2_tools/rosparam.hpp>
#include <tobas_tools/constants.hpp>
#include <tobas_tools/conversions/frame_id.hpp>
#include <tobas_msgs/RotorSpeeds.h>
#include <tobas_mr_mpc/ControllerFeedback.h>

#include "../include/tobas_mr_mpc/controller_ros.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_mr_mpc
{
ControllerRos::ControllerRos(, const string& name)
  : super(node, pnh, name),
    js_converter_(drone_.tree()),
    z_rotors_(drone_, tobas::Z_POSITIVE),
    acc_ctrl_(drone_),
    ori_ctrl_(drone_),
    server_(pnh_)
{
  getRosParams();
  drone_.loadFromParam(node_);

  z_rotors_.updateInternalDataStructures();
  js_converter_.updateInternalDataStructures();
  acc_ctrl_.updateInternalDataStructures();
  ori_ctrl_.updateInternalDataStructures();

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
  rot_speeds_pub_ = node_.advertise<tobas_msgs::RotorSpeeds>(tobas::kRotorSpeedsCmdTopic, 1);
  feedback_pub_ = node_.advertise<tobas_mr_mpc::ControllerFeedback>(tobas::kControllerFeedbackTopic, 1);
}

void ControllerRos::registerSubscribers()
{
  odom_sub_ = node_.subscribe(tobas::kOdometryTopic, 1, &self::odomCb, this, tcpNoDelay());
  battery_sub_ = node_.subscribe(tobas::kBatteryLpfTopic, 1, &self::batteryCb, this, tcpNoDelay());
  wind_sub_ = node_.subscribe(tobas::kWindTopic, 1, &self::windCb, this, tcpNoDelay());
  rotor_speeds_sub_ = node_.subscribe(tobas::kRotorSpeedsTopic, 1, &self::rotorSpeedsCb, this, tcpNoDelay());
  if (drone_.isTransformable())
    js_sub_ = node_.subscribe(tobas::kJointStatesTopic, 1, &self::jointStateCb, this, tcpNoDelay());
  if (do_thrust_correction_)
    thrust_corr_factor_sub_ =
      node_.subscribe(tobas::kThrustCorrectionFactorTopic, 1, &self::thrustCorrectionFactorCb, this, tcpNoDelay());
  arming_sub_ = node_.subscribe(tobas::kArmingTopic, 1, &self::armingCb, this, tcpNoDelay());

  pvay_sub_ = node_.subscribe(tobas::kPosVelAccYawCmdTopic, 1, &self::posVelAccYawCb, this, tcpNoDelay());
  rpyt_sub_ = node_.subscribe(tobas::kRpyThrustCmdTopic, 1, &self::rpyThrustCb, this, tcpNoDelay());
}

bool ControllerRos::isReadyToControl()
{
  if (odom_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kOdometryTopic);
    return false;
  }

  if (odom_->status != tobas_msgs::Odometry::NO_ERROR)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "There is a problem with the state estimation.");
    return false;
  }

  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kBatteryLpfTopic);
    return false;
  }

  if (wind_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kWindTopic);
    return false;
  }

  if (rotor_speeds_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(tobas::kCheckTopicsMsgPeriod, "Waiting for ", ns(), tobas::kRotorSpeedsTopic);
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

void ControllerRos::odomCb(const tobas_msgs::OdometryConstPtr& odom)
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
  const auto feedback = boost::make_shared<tobas_mr_mpc::ControllerFeedback>();
  feedback->header.stamp = odom->header.stamp;

  // Translation Controller
  if (tar_pvay_W_ != nullptr)
  {
    if (tar_rpyt_ == nullptr)
    {
      tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>();
    }

    // 世界座標系から見た現在の速度，加速度を計算
    const auto cur_vel_W = odom->frame.M * odom->twist.vel;
    const auto cur_acc_W = odom->frame.M * odom->accel.linear;

    // 目標加速度を計算
    pos_ctrl_.update(odom->frame.p, cur_vel_W, cur_acc_W, tar_pvay_W_->pos, tar_pvay_W_->vel, dt, tar_acc_fb_);
    const auto tar_acc = tar_pvay_W_->acc + tar_acc_fb_;

    // 推力和と目標姿勢を計算
    acc_ctrl_.update(
      odom->frame.M, odom->twist.vel, wind_->vel, rotor_speeds_->speeds, tar_acc, tar_rpyt_->thrust,
      tar_rpyt_->rpy.roll, tar_rpyt_->rpy.pitch);

    // コマンドレベルとヨー角は位置指令をそのまま流す
    tar_rpyt_->level = tar_pvay_W_->level;
    tar_rpyt_->rpy.yaw = tar_pvay_W_->yaw;

    // Fill feedback
    feedback->target_position = tar_pvay_W_->pos;
    feedback->target_velocity_global = tar_pvay_W_->vel;
    feedback->target_velocity_local = odom->frame.M.inverse(tar_pvay_W_->vel);
    feedback->target_acceleration_global = tar_acc;
    feedback->target_acceleration_local = odom->frame.M.inverse(tar_acc);
    feedback->position_integral_error.data = pos_ctrl_.positionIntegralError();
  }

  // Rotation Controller
  if (tar_rpyt_ != nullptr)
  {
    // 可動関節角を更新
    // 処理の遅延を防ぐため，JointStateのコールバックではなくここで行う
    if (drone_.isTransformable() && js_converter_.jointStateToJntArrayPos(*js_) < 0)
      TOBAS_ERROR("Joint state converter failed: ", js_converter_.errorMessage());

    // プロペラ推力を計算
    Eigen::VectorXd thrusts;
    try
    {
      // stopwatch_.start();
      thrusts = ori_ctrl_.solve(
        dt, odom->frame.M, odom->twist, wind_->vel, js_converter_.getPositionsKDL(), battery_->voltage,
        rotor_speeds_->speeds, tar_rpyt_->thrust, tar_rpyt_->rpy.toRotation());
      // stopwatch_.stop();
    }
    catch (const exception& e)
    {
      // MPCがコケたり．前回の推力をそのまま継続して印加する．
      fatal(e.what());
    }

    // 目標回転数を発行
    const auto tar_rot_speeds = boost::make_shared<tobas_msgs::RotorSpeeds>();
    tar_rot_speeds->header.stamp = odom->header.stamp;
    tar_rot_speeds->speeds.resize(drone_.numRotors(), 0.);
    for (size_t i = 0; i < static_cast<size_t>(thrusts.rows()); ++i)
    {
      auto thrust = max(0., thrusts(i));
      if (do_thrust_correction_ && thrust_corr_factor_ != nullptr)  // 推力補正
        thrust *= thrust_corr_factor_->data;
      tar_rot_speeds->speeds[z_rotors_.rotorIdx(i)] = z_rotors_.rotSpeedFromThrust(i, thrust);
    }
    rot_speeds_pub_.publish(tar_rot_speeds);

    // フィードバックを発行
    feedback->target_orientation = tar_rpyt_->rpy;
    feedback->target_thrust = tar_rpyt_->thrust;
    feedback->target_thrusts = eigen_tools::toStdVector(thrusts);
    feedback->mpc_thrusts = eigen_tools::toStdVector(ori_ctrl_.mpcThrusts());
    feedback_pub_.publish(feedback);
  }
}

void ControllerRos::batteryCb(const tobas_msgs::BatteryConstPtr& battery)
{
  battery_ = battery;
}

void ControllerRos::windCb(const tobas_msgs::WindConstPtr& wind)
{
  wind_ = wind;
}

void ControllerRos::rotorSpeedsCb(const tobas_msgs::RotorSpeedsConstPtr& rotor_speeds)
{
  rotor_speeds_ = rotor_speeds;
}

void ControllerRos::jointStateCb(const sensor_msgs::msg::JointStateConstPtr& js)
{
  if (js->name.size() != js->position.size())
  {
    TOBAS_ERROR("The size of joint name and position is different.");
    return;
  }

  js_ = js;
}

void ControllerRos::thrustCorrectionFactorCb(const std_msgs::Float64ConstPtr& msg)
{
  thrust_corr_factor_ = msg;
}

void ControllerRos::armingCb(const std_msgs::BoolConstPtr& arming)
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

void ControllerRos::posVelAccYawCb(const tobas_msgs::PosVelAccYawConstPtr& pvay)
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
  tar_pvay_W_ = boost::make_shared<tobas_msgs::PosVelAccYaw>(*pvay);

  // グローバル座標系に変換
  if (!tobas::changeFrame(tobas_msgs::FrameId::WORLD, odom_->frame.M, *tar_pvay_W_))
  {
    TOBAS_ERROR("Failed to change command frame. Probably the frame id is invalid.");
    tar_pvay_W_ = nullptr;
    return;
  }
}

void ControllerRos::rpyThrustCb(const tobas_msgs::RollPitchYawThrustConstPtr& rpyt)
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
  tar_rpyt_ = boost::make_shared<tobas_msgs::RollPitchYawThrust>(*rpyt);
}

void ControllerRos::dynamicReconfigureCb(const ConfigType& cfg, size_t)
{
  pos_cfg_.acc_delay_time_const = cfg.attitude_decay;  // 加速度の遅延 = 姿勢の遅延
  pos_cfg_.hor_pos_weight = cfg.horizontal_position_weight;
  pos_cfg_.ver_pos_weight = cfg.vertical_position_weight;
  pos_cfg_.hor_vel_weight = cfg.horizontal_velocity_weight;
  pos_cfg_.ver_vel_weight = cfg.vertical_velocity_weight;
  pos_cfg_.hor_acc_weight = cfg.horizontal_accel_weight;
  pos_cfg_.ver_acc_weight = cfg.vertical_accel_weight;
  pos_cfg_.hor_posint_weight = cfg.horizontal_position_integral_weight;
  pos_cfg_.ver_posint_weight = cfg.vertical_position_integral_weight;
  pos_cfg_.jerk_weight_log10 = cfg.jerk_weight_log10;
  pos_cfg_.max_hor_posint_error = cfg.max_horizontal_position_integral_error;
  pos_cfg_.max_ver_posint_error = cfg.max_vertical_position_integral_error;
  pos_cfg_.max_hor_acc = cfg.max_horizontal_accel;
  pos_cfg_.max_ver_acc = cfg.max_vertical_accel;
  pos_ctrl_.configure(pos_cfg_);

  acc_cfg_.max_attitude = cfg.max_attitude;
  // acc_cfg_.h_force_comp_rate = cfg.horizontal_force_compensation_rate;
  acc_cfg_.h_force_comp_rate = 0;  // FIXME: 並進EoMで風外乱を考慮するとオーバーシュートが過大
  acc_ctrl_.configure(acc_cfg_);

  ori_cfg_.h_force_comp_rate = cfg.horizontal_force_compensation_rate;
  ori_cfg_.kp = cfg.orientation_kp;
  ori_cfg_.kd = cfg.orientation_kd;
  ori_cfg_.pred_horizon = cfg.prediction_horizon;
  ori_cfg_.pred_steps = cfg.prediction_steps;
  ori_cfg_.attitude_decay = cfg.attitude_decay;
  ori_cfg_.heading_decay = cfg.heading_decay;
  ori_cfg_.angvel_decay = cfg.angular_velocity_decay;
  ori_cfg_.attitude_weight = cfg.attitude_weight;
  ori_cfg_.heading_weight = cfg.heading_weight;
  ori_cfg_.angvel_weight = cfg.angular_velocity_weight;
  ori_cfg_.thrust_rate_weight_log10 = cfg.thrust_rate_weight_log10;
  ori_ctrl_.configure(ori_cfg_);

  TOBAS_INFO("Dynamic parameters are updated.");
}
}  // namespace tobas_mr_mpc
