#include <algorithm>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_msgs/msg/rotor_state.hpp>
#include <tobas_gazebo_msgs/RotorDebug.h>

#include "./rotor_plugin.hpp"

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/time.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace gz;
namespace cmp = sim::components;

namespace gazebo
{
GazeboRotorPlugin::GazeboRotorPlugin() : super()
{
}

void GazeboRotorPlugin::Configure(
  const sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{initialize(sdf);
  // Get SDF parameters
  getSdfParams(sdf);

  // Add model error to the model constants
  addModelError();

  // Store the model entity
  model_ = model;

  // Get the pointer to the joint
  joint_ = model_->GetJoint(joint_name_);
  if (joint_ == nullptr)
    TOBAS_EXIT("Couldn't find specified joint \"" << joint_name_ << "\".");

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == nullptr)
    TOBAS_EXIT("Couldn't find specified link \"" << link_name_ << "\".");
  parent_link_ = link_->GetParentJointsLinks()[0];

  // Initialize the first order filter
  rotor_speed_filter_.initialize(time_const_up_, time_const_down_, 0.);

  // Register publishers and subscribers to the ROS master
  registerPubSub();

  // Listen to the update event
  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&self::onUpdate, this, _1));
}

void GazeboRotorPlugin::getSdfParams(const sdf::ElementPtr& sdf)
{
  getSdfParam(sdf, "motorNumber", motor_number_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "jointName", joint_name_);

  getSdfParam(sdf, "rotSpeedCoefficients", rot_speed_coefs_);
  if (rot_speed_coefs_.X() <= 0)
    TOBAS_EXIT("The first term of 'rotationSpeedCoefficients' must be positive.");
  if (rot_speed_coefs_.Y() < 0)
    TOBAS_EXIT("The second term of 'rotationSpeedCoefficients' must be non-negative.");

  getSdfParam(sdf, "motorConstant", motor_const_, NON_NEGATIVE);
  getSdfParam(sdf, "momentConstant", moment_const_, NON_NEGATIVE);
  getSdfParam(sdf, "rotorDragCoefficient", rotor_drag_coef_, NON_NEGATIVE);

  if (sdf->HasElement("turningDirection"))
  {
    const auto turning_direction = sdf->GetElement("turningDirection")->Get<string>();
    if (turning_direction == tobas::CW.name)
      direction_ = tobas::CW;
    else if (turning_direction == tobas::CCW.name)
      direction_ = tobas::CCW;
    else
      TOBAS_EXIT("Please only use 'CW' or 'CCW' as turningDirection.");
  }
  else
  {
    TOBAS_EXIT("Please specify a turning direction ('CW' or 'CCW').");
  }

  getSdfParam(sdf, "timeConstantUp", time_const_up_, POSITIVE);
  getSdfParam(sdf, "timeConstantDown", time_const_down_, POSITIVE);
  if (time_const_up_ > kTimeConstWarnThreshold)
    gzwarn << "The value provided for 'timeConstantUp' appears to be too large: " << time_const_up_
           << "[s]. Please check settings and datasheet." << endl;
  if (time_const_down_ > kTimeConstWarnThreshold)
    gzwarn << "The value provided for 'timeConstantDown' appears to be too large: " << time_const_down_
           << "[s]. Please check settings and datasheet." << endl;

  getSdfParam(sdf, "maxRotationSpeed", max_rot_speed_, POSITIVE);

  getSdfParam(sdf, "numPoles", num_poles_);
  if (num_poles_ <= 0)
    TOBAS_EXIT("The number of poles must be positive.");
  if (num_poles_ % 2 != 0)
    TOBAS_EXIT("The number of poles must be even.");

  getSdfParam(sdf, "maxCurrent", max_current_, POSITIVE);

  string esc_mode;
  getSdfParam(sdf, "escMode", esc_mode);
  if (esc_mode == tobas::BLHELI_OPEN_LOOP.name)
    esc_mode_ = tobas::BLHELI_OPEN_LOOP;
  else if (esc_mode == tobas::BLHELI_CLOSED_LOOP_LOW_RANGE.name)
    esc_mode_ = tobas::BLHELI_CLOSED_LOOP_LOW_RANGE;
  else if (esc_mode == tobas::BLHELI_CLOSED_LOOP_MID_RANGE.name)
    esc_mode_ = tobas::BLHELI_CLOSED_LOOP_MID_RANGE;
  else if (esc_mode == tobas::BLHELI_CLOSED_LOOP_HIGH_RANGE.name)
    esc_mode_ = tobas::BLHELI_CLOSED_LOOP_HIGH_RANGE;
  else
    TOBAS_EXIT("Invalid ESC signal mode: " << esc_mode);

  getSdfParam(sdf, "maxModelErrorRate", max_model_error_rate_, kDefaultMaxModelErrorRate, NON_NEGATIVE);
}

void GazeboRotorPlugin::PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager& ecm)
{
  // Check topics
  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(kWarnPeriod, "/" << ns() << "/" << kBatteryGtTopic << " is not received yet.");
    return;
  }
  if (!wind_received_)
  {
    TOBAS_WARN_THROTTLE(kWarnPeriod, "/" << ns() << "/" << kWindGtTopic << " is not received yet.");
    return;
  }

  // 最後にスロットルコマンドが指令された時刻から一定時間経過したら強制的にDisarmする
  const auto& cur_time = info.simTime;
  const auto time_after_last_cmd = cur_time - last_cmd_time_;
  if (time_after_last_cmd > kCommandBlankTimeThreshold)
  {
    cmd_rot_speed_ = 0.;
    disarm_start_time_ = cur_time;
    is_armed_ = false;
  }

  // Get rotation speed
  const auto rot_speed_sim = joint_->GetVelocity(0);
  const auto rot_speed_real = rot_speed_sim * kRotorSpeedSlowdownSim;

  // Compute time after previous simulation time
  const auto dt = (cur_time - prev_sim_time_).Double();
  prev_sim_time_ = cur_time;

  // Check aliasing
  if (abs(rot_speed_sim) * dt > M_PI)
  {
    TOBAS_WARN_THROTTLE(
      kWarnPeriod, "Aliasing on motor [" << motor_number_ << "] might occur. Lower simulation time step.");
  }

  // Update simulation state
  applyForceAndTorque(rot_speed_real, info.simTime);

  // ESCが壊れていなければ回転数を更新
  if (is_intact_)
    updateRotationSpeed(dt);
}

void GazeboRotorPlugin::registerPubSub()
{
  const string suffix = "_" + to_string(motor_number_);

  rotor_state_pub_ = createPublisher<tobas_msgs::msg::RotorState>(path::join(ns(), kRotorStateGtTopicPrefix + suffix));
  debug_pub_ = createPublisher<tobas_gazebo_msgs::RotorDebug>(path::join(ns(), kDebugTopicPrefix + suffix));

  throttle_sub_ = createSubscriber(
    path::join(ns(), kThrottleTopicPrefix + suffix), 1, &self::throttleCmdCb, this,
    rclcpp::TransportHints().tcpNoDelay());
  battery_gt_sub_ = createSubscriber(
    path::join(ns(), kBatteryGtTopic), &self::batteryGtCb, this, rclcpp::TransportHints().tcpNoDelay());
  wind_gt_sub_ =
    createSubscriber(path::join(ns(), kWindGtTopic), &self::windSpeedGtCb, this, rclcpp::TransportHints().tcpNoDelay());
}

bool GazeboRotorPlugin::isReady()
{
  return battery_ != nullptr && wind_received_;
}

void GazeboRotorPlugin::addModelError()
{
  // 一様乱数を作成
  random_device rnd_dev;
  mt19937 rnd_gen(rnd_dev());
  UniformDistribution uniform(-1, 1);

  // 回転数-電圧の関係式
  // 1次の係数はKv値から概ね正確な値が分かるため，2次の係数にのみ誤差を加える．
  rot_speed_coefs_.Y() *= (1 + max_model_error_rate_ * uniform(rnd_gen));

  // 空力定数
  motor_const_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
  moment_const_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
  rotor_drag_coef_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
}

void GazeboRotorPlugin::applyForceAndTorque(const double& rot_speed, const chrono::steady_clock::duration& cur_time)
{
  // The True Role of Accelerometer Feedback in Quadrotor Control [Martin+, 2010]
  // II-A. Model of a single propeller near hovering
  // TODO: Implement other terms
  // TODO: II-B. Model of the complete quadrotor

  // Get joint axes
  const auto global_axis = joint_->GlobalAxis(0);
  const auto local_axis = joint_->LocalAxis(0);

  // (1) first term: Thrust Force
  const auto rot_speed_sgn = math::sign(rot_speed);
  const auto thrust = direction_.value * rot_speed_sgn * motor_const_ * math::sqr(rot_speed);
  const auto thrust_W = thrust * global_axis;
  link_->AddForce(thrust_W);

  // (1) second term: H-force
  const auto linvel_W = link_->WorldLinearVel() - wind_vel_W_;
  const auto linvel_perp_W = linvel_W - (linvel_W.Dot(global_axis) * global_axis);
  const auto h_force_W = (-abs(rot_speed) * rotor_drag_coef_) * linvel_perp_W;
  link_->AddForce(h_force_W);

  // (2) first term: Rotor drag torque
  const auto pose_diff = link_->WorldCoGPose() - parent_link_->WorldCoGPose();
  const auto torque = moment_const_ * thrust;
  const auto drag_torque_child = (-direction_.value * torque) * local_axis;
  const auto drag_torque_parent = pose_diff.Rot().RotateVector(drag_torque_child);
  parent_link_->AddRelativeTorque(drag_torque_parent);

  // Compute electric current
  const auto& kt = rot_speed_coefs_.X();  // トルク定数 = 発電係数 = Kvの逆数 (内部抵抗値に依らない)
  const auto current = torque / kt;

  // 安全のため，一瞬でも過電流が流れたらESCが焼き切れたとみなす
  if (current > max_current_)
  {
    gzerr << "The ESC of rotor " << motor_number_ << " is critically damaged due to an overcurrent of " << current
          << " A, which exceeded its maximum current capacity of " << max_current_ << " A." << endl;
    joint_->SetVelocity(0, 0.);
    is_intact_ = false;
  }

  // Publish rotor state
  const auto rotor_state = std::make_unique<tobas_msgs::msg::RotorState>();
  ros2::timeChronoToMsg(cur_time, rotor_state->header.stamp);
  rotor_state->speed = rot_speed;
  rotor_state->current = current;
  rotor_state_pub_->publish(rotor_state);

  // Publish debug message
  const auto debug_msg = std::make_unique<tobas_gazebo_msgs::RotorDebug>();
  ros2::timeChronoToMsg(cur_time, debug_msg->header.stamp);
  debug_msg->rotation_speed = joint_->GetVelocity(0) * kRotorSpeedSlowdownSim;
  vectorGazeboToKDL(thrust_W, debug_msg->thrust_force);
  vectorGazeboToKDL(h_force_W, debug_msg->horizontal_force);
  vectorGazeboToKDL(drag_torque_parent, debug_msg->drag_torque);
  debug_pub_->publish(debug_msg);
}

void GazeboRotorPlugin::updateRotationSpeed(const double& dt)
{
  assert(dt >= 0);

  // アクティベートされていなければ無回転
  if (!is_armed_)
  {
    joint_->SetVelocity(0, 0.);
    return;
  }

  // Check rotor speed limit and get set value
  auto set_rot_speed = cmd_rot_speed_;
  const auto max_rot_speed = min(max_rot_speed_, rotSpeedFromVoltage(battery_->voltage));
  if (cmd_rot_speed_ < 0)
  {
    gzerr << "Negative rotor speed is commanded on index " << motor_number_ << ": " << cmd_rot_speed_ << " < 0 [rad/s]"
          << endl;
    set_rot_speed = 0.;
  }
  else if (cmd_rot_speed_ > max_rot_speed + kRotorSpeedCheckMargin)
  {
    GZ_ERROR_THROTTLE(
      kErrorPeriod, "Target rotor speed on index " << motor_number_ << " is too high: " << cmd_rot_speed_ << " > "
                                                   << max_rot_speed << " [rad/s]");
    set_rot_speed = max_rot_speed;
  }

  // Apply the filter on the rotation speed
  const auto ref_rot_speed = rotor_speed_filter_.update(set_rot_speed, dt);
  joint_->SetVelocity(0, direction_.value * ref_rot_speed / kRotorSpeedSlowdownSim);
}

double GazeboRotorPlugin::rotSpeedFromVoltage(const double& voltage)
{
  const auto& a = rot_speed_coefs_.X();
  const auto& b = rot_speed_coefs_.Y();
  return b > 0 ? (sqrt(math::sqr(a) + 4 * b * voltage) - a) / (2 * b) : voltage / a;
}

double GazeboRotorPlugin::rotSpeedFromERPM(const double& erpm)
{
  return tobas_std::rpm2rps(erpm * 2 / num_poles_);
}

void GazeboRotorPlugin::throttleCmdCb(const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& msg)
{
  // バッテリーの情報が無いか電圧が低すぎたら応答なし
  if (battery_ == nullptr || battery_->voltage < kMinBatteryVoltage)
    return;

  // 最後にコマンドを受け取った時刻を更新
  last_cmd_time_ = prev_sim_time_;

  if (is_armed_)
  {
    // スロットルの範囲を制限
    const auto throttle = std::clamp(msg->data, tobas::kMinThrottle, tobas::kMaxThrottle);

    // スロットルを目標回転数に変換
    switch (esc_mode_.value)
    {
      case tobas::BLHELI_OPEN_LOOP.value:
      {
        const auto input_voltage =
          math::remap(throttle, tobas::kMinThrottle, tobas::kMaxThrottle, 0., battery_->voltage);
        cmd_rot_speed_ = rotSpeedFromVoltage(input_voltage);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_LOW_RANGE.value:
      {
        const auto erpm =
          math::remap(throttle, tobas::kMinThrottle, tobas::kMaxThrottle, 0., tobas::kBLHeliCLLowMaxERPM);
        cmd_rot_speed_ = rotSpeedFromERPM(erpm);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_MID_RANGE.value:
      {
        const auto erpm =
          math::remap(throttle, tobas::kMinThrottle, tobas::kMaxThrottle, 0., tobas::kBLHeliCLMidMaxERPM);
        cmd_rot_speed_ = rotSpeedFromERPM(erpm);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_HIGH_RANGE.value:
      {
        const auto erpm =
          math::remap(throttle, tobas::kMinThrottle, tobas::kMaxThrottle, 0., tobas::kBLHeliCLHighMaxERPM);
        cmd_rot_speed_ = rotSpeedFromERPM(erpm);
        break;
      }
      default:
      {
        throw;
      }
    }
  }
  else
  {
    // 最小スロットルでなければDisarm開始時刻をリセット
    if (msg->data > tobas::kMinThrottle)
      disarm_start_time_ = prev_sim_time_;

    // Disarmの時間が一定時間を超えたらArmする
    if ((prev_sim_time_ - disarm_start_time_).Double() > kDisarmDuration)
    {
      is_armed_ = true;
      gzmsg << "Rotor " << motor_number_ << " is armed." << endl;
    }
  }
}

void GazeboRotorPlugin::batteryGtCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  battery_ = battery;
}

void GazeboRotorPlugin::windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind)
{
  vectorKDLToGazebo(wind->vel, wind_vel_W_);

  if (!wind_received_)
    wind_received_ = true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRotorPlugin);
}  // namespace gazebo
