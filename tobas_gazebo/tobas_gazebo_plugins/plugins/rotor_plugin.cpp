#include <algorithm>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/turning_direction.hpp>
#include <tobas_drone_core/esc.hpp>
#include <tobas_msgs/msg/rotor_state.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/Wind.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_gazebo_msgs/msg/rotor_debug.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/first_order_filter.hpp"

using namespace std;
using namespace std::chrono;
using namespace gz;
using namespace gz::math;
namespace cmp = sim::components;

namespace gazebo
{
/* Simulates ESC, rotor and ropeller. */
class GazeboRotorPlugin : public BaseNode,
                          public sim::System,
                          public sim::ISystemConfigure,
                          public sim::ISystemPreUpdate
{
  // Constants
  static constexpr char kDebugTopicPrefix[] = "gazebo/rotor_debug";
  static constexpr double kRotorSpeedCheckMargin = 10.;     // [rad/s]
  static constexpr double kCommandBlankTimeThreshold = 1.;  // [s]
  static constexpr double kTimeConstWarnThreshold = 0.1;    // [s]
  static constexpr double kMinBatteryVoltage = 3.;          // [V]
  static constexpr double kDisarmDuration = 1.5;            // [s] 通常1~2秒らしい

  // Default parameters
  static constexpr double kDefaultMaxModelErrorRate = 0.;

  using self = GazeboRotorPlugin;

public:
  explicit GazeboRotorPlugin();

  void Configure(
    const sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    sim::EntityComponentManager& ecm,
    sim::EventManager&) override;

  void PreUpdate(const sim::UpdateInfo& info, sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  size_t channel_;
  string joint_name_;
  Vector2d rot_speed_coefs_;  // [Vs/rad, (Vs/rad)^2]
  double motor_const_;
  double moment_const_;
  double rotor_drag_coef_;
  int direction_;  // Turning direction: 1(CCW) or -1(CW).
  double time_const_up_;
  double time_const_down_;
  double max_rot_speed_;  // [rad/s] 最大連続電流によって定まるモータ特性が成り立つ最大回転数
  size_t num_poles_;      // モータの極数
  double max_current_;    // [A] ESCの最大電流
  tobas::esc_mode_t esc_mode_;  // ESCへの信号の解釈方式
  double max_model_error_rate_;

  double cmd_rot_speed_ = 0.;  // [rad/s]
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  Vector3d wind_vel_W_ = Vector3d::Zero;  // [m/s]
  steady_clock::duration prev_sim_time_;
  steady_clock::duration last_cmd_time_;  // 最後にスロットルコマンドが指令された時刻
  steady_clock::duration disarm_start_time_ = steady_clock::duration::max();  // Disarmコマンドの開始時刻
  bool is_intact_ = true;
  bool is_armed_ = false;
  bool wind_received_ = false;
  AsymmetricFirstOrderFilter<double> rotor_speed_filter_;

  // Gazebo objects
  shared_ptr<sim::Joint> joint_;
  shared_ptr<sim::Link> link_;
  shared_ptr<sim::Link> parent_link_;

  // PubSub
  PublisherPtr<tobas_msgs::msg::RotorState> rotor_state_pub_;
  PublisherPtr<tobas_gazebo_msgs::msg::RotorDebug> debug_pub_;
  SubscriberPtr<tobas_gazebo_msgs::msg::Throttle> throttle_sub_;
  SubscriberPtr<tobas_msgs::msg::Battery> battery_gt_sub_;
  SubscriberPtr<tobas_msgs::Wind> wind_gt_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

  void registerPubSub();
  void addModelError();
  void applyWrench(sim::EntityComponentManager& ecm, const double& rot_speed, const steady_clock::duration& cur_time);
  void updateRotationSpeed(sim::EntityComponentManager& ecm, const double& dt);
  double rotSpeedFromVoltage(const double& voltage);
  double rotSpeedFromERPM(const double& erpm);

  void throttleCmdCb(const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& msg);
  void batteryGtCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind);
};

GazeboRotorPlugin::GazeboRotorPlugin() : BaseNode("rotor_plugin")
{
}

void GazeboRotorPlugin::Configure(
  const sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{
  initialize(sdf);
  getSdfParams(sdf);
  addModelError();

  rotor_speed_filter_.initialize(time_const_up_, time_const_down_, 0.);

  // Get robot model
  sim::Model model(model_entity);
  if (!model.Valid(ecm))
    TOBAS_EXIT("Failed to find model.");

  // Get joint
  const auto joint_entity = model.JointByName(ecm, joint_name_);
  joint_ = make_shared<sim::Joint>(joint_entity);
  if (!joint_->Valid(ecm))
    TOBAS_EXIT("Failed to find specified joint \"", joint_name_, "\".");

  // Check joint type
  const auto joint_type = joint_->Type(ecm).value();
  if (joint_type != sdf::JointType::CONTINUOUS && joint_type != sdf::JointType::REVOLUTE)
    TOBAS_EXIT("Joint \"", joint_name_, "\" is not a rotating joint.");

  // Get child link
  const auto link_name = joint_->ChildLinkName(ecm).value();
  const auto link_entity = model.LinkByName(ecm, link_name);
  link_ = make_shared<sim::Link>(link_entity);
  if (!link_->Valid(ecm))
    TOBAS_EXIT("Failed to find the child link \"", link_name, "\".");

  // Get parent link
  const auto parent_link_name = joint_->ChildLinkName(ecm).value();
  const auto parent_link_entity = model.LinkByName(ecm, parent_link_name);
  parent_link_ = make_shared<sim::Link>(parent_link_entity);
  if (!parent_link_->Valid(ecm))
    TOBAS_EXIT("Failed to find the parent link \"", parent_link_name, "\".");

  // Create necessary components
  if (!getComponent<cmp::JointAxis>(joint_entity, ecm))
    TOBAS_EXIT("Failed to get component JointAxis to joint \"", joint_name_, "\".");
  if (!getComponent<cmp::JointVelocity>(joint_entity, ecm))
    TOBAS_EXIT("Failed to get component JointVelocity to joint \"", joint_name_, "\".");
  if (!getComponent<cmp::WorldPose>(link_entity, ecm))
    TOBAS_EXIT("Failed to get component WorldPose to link \"", link_name, "\".");
  if (!getComponent<cmp::WorldLinearVelocity>(link_entity, ecm))
    TOBAS_EXIT("Failed to get component WorldLinearVelocity to link \"", link_name, "\".");

  // Register publishers and subscribers
  registerPubSub();
}

void GazeboRotorPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "motorNumber", channel_);
  getSdfParam(sdf, "jointName", joint_name_);

  getSdfParam(sdf, "rotSpeedCoefficients", rot_speed_coefs_);
  if (rot_speed_coefs_.X() <= 0)
    TOBAS_EXIT("The first term of 'rotationSpeedCoefficients' must be positive.");
  if (rot_speed_coefs_.Y() < 0)
    TOBAS_EXIT("The second term of 'rotationSpeedCoefficients' must be non-negative.");

  getSdfParam(sdf, "motorConstant", motor_const_, NON_NEGATIVE);
  getSdfParam(sdf, "momentConstant", moment_const_, NON_NEGATIVE);
  getSdfParam(sdf, "rotorDragCoefficient", rotor_drag_coef_, NON_NEGATIVE);

  string turning_direction;
  getSdfParam(sdf, "turningDirection", turning_direction);
  if (turning_direction == tobas::turning_direction::kCCWName)
    direction_ = 1;
  else if (turning_direction == tobas::turning_direction::kCWName)
    direction_ = -1;
  else
    TOBAS_EXIT("Please specify 'CW' or 'CCW' as turningDirection.");

  getSdfParam(sdf, "timeConstantUp", time_const_up_, POSITIVE);
  getSdfParam(sdf, "timeConstantDown", time_const_down_, POSITIVE);
  if (time_const_up_ > kTimeConstWarnThreshold)
    TOBAS_WARN(
      "The value provided for 'timeConstantUp' appears to be too large: ", time_const_up_,
      "[s]. Please check settings and datasheet.");
  if (time_const_down_ > kTimeConstWarnThreshold)
    TOBAS_WARN(
      "The value provided for 'timeConstantDown' appears to be too large: ", time_const_down_,
      "[s]. Please check settings and datasheet.");

  getSdfParam(sdf, "maxRotationSpeed", max_rot_speed_, POSITIVE);

  getSdfParam(sdf, "numPoles", num_poles_);
  if (num_poles_ <= 0)
    TOBAS_EXIT("The number of poles must be positive.");
  if (num_poles_ % 2 != 0)
    TOBAS_EXIT("The number of poles must be even.");

  getSdfParam(sdf, "maxCurrent", max_current_, POSITIVE);

  string esc_mode;
  getSdfParam(sdf, "escMode", esc_mode);
  if (esc_mode == tobas::esc::kBLHeliOpenLoopName)
    esc_mode_ = tobas::BLHELI_OPEN_LOOP;
  else if (esc_mode == tobas::esc::kBLHeliCloseLoopLowName)
    esc_mode_ = tobas::BLHELI_CLOSED_LOOP_LOW_RANGE;
  else if (esc_mode == tobas::esc::kBLHeliCloseLoopMidName)
    esc_mode_ = tobas::BLHELI_CLOSED_LOOP_MID_RANGE;
  else if (esc_mode == tobas::esc::kBLHeliCloseLoopHighName)
    esc_mode_ = tobas::BLHELI_CLOSED_LOOP_HIGH_RANGE;
  else
    TOBAS_EXIT("Invalid ESC signal mode: ", esc_mode);

  getSdfParam(sdf, "maxModelErrorRate", max_model_error_rate_, kDefaultMaxModelErrorRate, NON_NEGATIVE);
}

void GazeboRotorPlugin::PreUpdate(const sim::UpdateInfo& info, sim::EntityComponentManager& ecm)
{
  // Update the previous simulation step time
  prev_sim_time_ = info.simTime;

  // Check topics
  if (battery_ == nullptr)
  {
    TOBAS_WARN_THROTTLE(kWarnPeriod, kBatteryGtTopic, " is not received yet.");
    return;
  }
  if (!wind_received_)
  {
    TOBAS_WARN_THROTTLE(kWarnPeriod, kWindGtTopic, " is not received yet.");
    return;
  }

  // 最後にスロットルコマンドが指令された時刻から一定時間経過したら強制的にDisarmする
  const auto time_after_last_cmd = chrono::duration<double>(info.simTime - last_cmd_time_).count();
  if (time_after_last_cmd > kCommandBlankTimeThreshold)
  {
    cmd_rot_speed_ = 0.;
    disarm_start_time_ = info.simTime;
    is_armed_ = false;
  }

  // Get rotation speed
  const auto rot_speed_sim = joint_->Velocity(ecm).value().at(0);
  const auto rot_speed_real = rot_speed_sim * kRotorSpeedSlowdownSim;

  // Compute time after previous simulation time
  const auto dt = chrono::duration<double>(info.dt).count();

  // Check aliasing
  if (abs(rot_speed_sim) * dt > M_PI)
    TOBAS_WARN_THROTTLE(kWarnPeriod, "Aliasing on motor [", channel_, "] might occur. Lower simulation time step.");

  // Update simulation state
  applyWrench(ecm, rot_speed_real, info.simTime);

  // ESCが壊れていなければ回転数を更新
  if (is_intact_)
    updateRotationSpeed(ecm, dt);
}

void GazeboRotorPlugin::registerPubSub()
{
  const string suffix = "_" + to_string(channel_);

  rotor_state_pub_ = createPublisher<tobas_msgs::msg::RotorState>(path::join(ns(), kRotorStateGtTopicPrefix + suffix));
  debug_pub_ = createPublisher<tobas_gazebo_msgs::msg::RotorDebug>(path::join(ns(), kDebugTopicPrefix + suffix));

  throttle_sub_ = createSubscriber(path::join(ns(), kThrottleTopicPrefix + suffix), &self::throttleCmdCb, this);
  battery_gt_sub_ = createSubscriber(path::join(ns(), kBatteryGtTopic), &self::batteryGtCb, this);
  wind_gt_sub_ = createSubscriber(path::join(ns(), kWindGtTopic), &self::windSpeedGtCb, this);
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

void GazeboRotorPlugin::applyWrench(
  sim::EntityComponentManager& ecm,
  const double& rot_speed,
  const steady_clock::duration& cur_time)
{
  // The True Role of Accelerometer Feedback in Quadrotor Control [Martin+, 2010]
  // II-A. Model of a single propeller near hovering
  // TODO: Implement other terms
  // TODO: II-B. Model of the complete quadrotor

  // Get joint axes
  const auto& local_axis = joint_->Axis(ecm).value().at(0).Xyz();
  const auto global_axis = link_->WorldPose(ecm).value().Rot().RotateVector(local_axis);

  // (1) first term: Thrust Force
  const auto rot_speed_sgn = ::math::sign(rot_speed);
  const auto thrust = direction_ * rot_speed_sgn * motor_const_ * ::math::sqr(rot_speed);
  const auto thrust_W = thrust * global_axis;
  link_->AddWorldWrench(ecm, thrust_W, Vector3d::Zero);

  // (1) second term: H-force
  const auto linvel_W = link_->WorldLinearVelocity(ecm).value() - wind_vel_W_;
  const auto linvel_perp_W = linvel_W - (linvel_W.Dot(global_axis) * global_axis);
  const auto h_force_W = (-abs(rot_speed) * rotor_drag_coef_) * linvel_perp_W;
  link_->AddWorldWrench(ecm, h_force_W, Vector3d::Zero);

  // (2) first term: Rotor drag torque
  const auto torque = moment_const_ * thrust;
  const auto drag_torque_W = (-direction_ * torque) * global_axis;
  parent_link_->AddWorldWrench(ecm, Vector3d::Zero, drag_torque_W);

  // Compute electric current
  const auto& kt = rot_speed_coefs_.X();  // トルク定数 = 発電係数 = Kvの逆数 (内部抵抗値に依らない)
  const auto current = torque / kt;

  // 安全のため，一瞬でも過電流が流れたらESCが焼き切れたとみなす
  if (current > max_current_)
  {
    TOBAS_ERROR(
      "The ESC of rotor ", channel_, " is critically damaged due to an overcurrent of ", current,
      " A, which exceeded its maximum current capacity of ", max_current_, " A.");
    joint_->SetVelocity(ecm, { 0. });
    is_intact_ = false;
  }

  // Publish rotor state
  auto rotor_state = make_unique<tobas_msgs::msg::RotorState>();
  ros2::timeChronoToMsg(cur_time, rotor_state->header.stamp);
  rotor_state->speed = rot_speed;
  rotor_state->current = current;
  rotor_state_pub_->publish(move(rotor_state));

  // Publish debug message
  auto debug_msg = make_unique<tobas_gazebo_msgs::msg::RotorDebug>();
  ros2::timeChronoToMsg(cur_time, debug_msg->header.stamp);
  debug_msg->rotation_speed = joint_->Velocity(ecm).value().at(0) * kRotorSpeedSlowdownSim;
  vectorGazeboToMsg(thrust_W, debug_msg->thrust_force);
  vectorGazeboToMsg(h_force_W, debug_msg->horizontal_force);
  vectorGazeboToMsg(drag_torque_W, debug_msg->drag_torque);
  debug_pub_->publish(move(debug_msg));
}

void GazeboRotorPlugin::updateRotationSpeed(sim::EntityComponentManager& ecm, const double& dt)
{
  assert(dt >= 0);

  // アクティベートされていなければ無回転
  if (!is_armed_)
  {
    joint_->SetVelocity(ecm, { 0. });
    return;
  }

  // Check rotor speed limit and get set value
  auto set_rot_speed = cmd_rot_speed_;
  const auto max_rot_speed = min(max_rot_speed_, rotSpeedFromVoltage(battery_->voltage));
  if (cmd_rot_speed_ < 0)
  {
    TOBAS_ERROR("Negative rotor speed is commanded on index ", channel_, ": ", cmd_rot_speed_, " < 0 [rad/s]");
    set_rot_speed = 0.;
  }
  else if (cmd_rot_speed_ > max_rot_speed + kRotorSpeedCheckMargin)
  {
    TOBAS_ERROR_THROTTLE(
      kErrorPeriod, "Target rotor speed on index ", channel_, " is too high: ", cmd_rot_speed_, " > ", max_rot_speed,
      " [rad/s]");
    set_rot_speed = max_rot_speed;
  }

  // Apply the filter on the rotation speed
  const auto ref_rot_speed = rotor_speed_filter_.update(set_rot_speed, dt);
  joint_->SetVelocity(ecm, { direction_ * ref_rot_speed / kRotorSpeedSlowdownSim });
}

double GazeboRotorPlugin::rotSpeedFromVoltage(const double& voltage)
{
  const auto& a = rot_speed_coefs_.X();
  const auto& b = rot_speed_coefs_.Y();
  return b > 0 ? (sqrt(::math::sqr(a) + 4 * b * voltage) - a) / (2 * b) : voltage / a;
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
    const auto throt = std::clamp(msg->data, tobas::kMinThrot, tobas::kMaxThrot);

    // スロットルを目標回転数に変換
    switch (esc_mode_)
    {
      case tobas::BLHELI_OPEN_LOOP:
      {
        const auto input_voltage = ::math::remap(throt, tobas::kMinThrot, tobas::kMaxThrot, 0., battery_->voltage);
        cmd_rot_speed_ = rotSpeedFromVoltage(input_voltage);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_LOW_RANGE:
      {
        const auto erpm = ::math::remap(throt, tobas::kMinThrot, tobas::kMaxThrot, 0., tobas::esc::kBLHeliCLLowMaxERPM);
        cmd_rot_speed_ = rotSpeedFromERPM(erpm);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_MID_RANGE:
      {
        const auto erpm = ::math::remap(throt, tobas::kMinThrot, tobas::kMaxThrot, 0., tobas::esc::kBLHeliCLMidMaxERPM);
        cmd_rot_speed_ = rotSpeedFromERPM(erpm);
        break;
      }
      case tobas::BLHELI_CLOSED_LOOP_HIGH_RANGE:
      {
        const auto erpm =
          ::math::remap(throt, tobas::kMinThrot, tobas::kMaxThrot, 0., tobas::esc::kBLHeliCLHighMaxERPM);
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
    if (msg->data > tobas::kMinThrot)
      disarm_start_time_ = prev_sim_time_;

    // Disarmの時間が一定時間を超えたらArmする
    if (chrono::duration<double>(prev_sim_time_ - disarm_start_time_).count() > kDisarmDuration)
    {
      is_armed_ = true;
      TOBAS_INFO("Rotor ", channel_, " is armed.");
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
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboRotorPlugin,
  sim::System,
  gazebo::GazeboRotorPlugin::ISystemConfigure,
  gazebo::GazeboRotorPlugin::ISystemPreUpdate)
