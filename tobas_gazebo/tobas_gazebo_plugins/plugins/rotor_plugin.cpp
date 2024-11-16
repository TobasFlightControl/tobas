#include <algorithm>
#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_drone_core/turning_direction.hpp>
#include <tobas_msgs/msg/rotor_state.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs_adapter/Wind.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_gazebo_msgs/msg/rotor_debug.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

// モータのインダクタンスが不明なことが多いため，Kvとの積が概ね一定になることを利用する．
// 実機の時定数がシミュレーションよりも大きくならないように想定しうる最大値に設定する．
#define L_KV 0.02

using namespace std;
using namespace chrono;
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
  static constexpr char kDebugTopicPrefix[] = "gazebo/rotor_debug_";
  static constexpr double kRotorSpeedCheckMargin = 10.;   // [rad/s]
  static constexpr double kAutoStopTimeThresh = 0.5;      // [s]
  static constexpr double kTimeConstWarnThreshold = 0.1;  // [s]
  static constexpr double kMinBatteryVoltage = 3.;        // [V]

  // Default parameters
  static constexpr double kDefaultMaxModelErrorRate = 0.;

  using self = GazeboRotorPlugin;
  using BreakSrv = std_srvs::srv::Trigger;

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
  string joint_name_;
  size_t channel_;
  double kv_;               // [rad/s/V]
  double resistance_;       // [Ω]
  double motor_const_;      // [N/(rad/s)^2]
  double moment_const_;     // [m]
  double rotor_drag_coef_;  // [N*s^2/rad/m]
  int direction_;           // Turning direction: 1(CCW) or -1(CW).
  double max_current_;      // [A] ESCの最大電流
  double max_model_error_rate_;

  double throttle_ = 0.;  // [0, 1]
  tobas_msgs::msg::Battery::ConstSharedPtr battery_;
  Vector3d wind_vel_W_ = Vector3d::Zero;  // [m/s]
  steady_clock::duration prev_sim_time_;
  steady_clock::duration last_cmd_time_;  // 最後にスロットルコマンドが指令された時刻
  bool is_intact_ = true;
  bool wind_received_ = false;

  // Gazebo objects
  shared_ptr<sim::Joint> joint_;
  shared_ptr<sim::Link> link_;
  shared_ptr<sim::Link> parent_link_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorState> state_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::RotorState> state_gt_pub_;
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::RotorDebug> debug_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::Throttle> throttle_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_gt_sub_;
  ros2::SubscriberPtr<tobas_msgs::Wind> wind_gt_sub_;

  // Services
  ros2::ServiceServerPtr<BreakSrv> break_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);

  void registerROSInterfaces();
  void addModelError();
  void applyWrench(sim::EntityComponentManager& ecm, double rot_speed, const steady_clock::duration& cur_time);
  void updateRotationSpeed(sim::EntityComponentManager& ecm, double rot_speed, double dt);

  void throttleCmdCb(const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& msg);
  void batteryGtCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery);
  void windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind);

  void breakCb(const BreakSrv::Request::ConstSharedPtr& req, const BreakSrv::Response::SharedPtr& res);
};

GazeboRotorPlugin::GazeboRotorPlugin()
{
}

void GazeboRotorPlugin::Configure(
  const sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager& ecm,
  sim::EventManager&)
{
  getSdfParams(sdf);
  initialize("gazebo_rotor_plugin_" + to_string(channel_), sdf);
  addModelError();

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

  // Register ROS interfaces
  registerROSInterfaces();
}

void GazeboRotorPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "jointName", joint_name_);
  getSdfParam(sdf, "channel", channel_);

  getSdfParam(sdf, "kv", kv_, POSITIVE);
  getSdfParam(sdf, "internalResistance", resistance_, POSITIVE);

  getSdfParam(sdf, "motorConstant", motor_const_, POSITIVE);
  getSdfParam(sdf, "momentConstant", moment_const_, POSITIVE);
  getSdfParam(sdf, "rotorDragCoefficient", rotor_drag_coef_, NON_NEGATIVE);

  int turning_direction;
  getSdfParam(sdf, "turningDirection", turning_direction);
  direction_ = tobas::sign(static_cast<tobas::turning_direction_t>(turning_direction));

  getSdfParam(sdf, "maxCurrent", max_current_, POSITIVE);
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

  // 最後にスロットルコマンドが指令された時刻から一定時間経過したら強制的にモータを停止する
  const auto secs_from_last_cmd = duration<double>(info.simTime - last_cmd_time_).count();
  if (secs_from_last_cmd > kAutoStopTimeThresh)
    throttle_ = 0.;

  // Get rotation speed
  const auto rot_speed_sim = max(joint_->Velocity(ecm).value().at(0) * direction_, 0.);
  const auto rot_speed_real = rot_speed_sim * kRotorSpeedSlowdownSim;

  // Compute time after previous simulation time
  const auto dt = duration<double>(info.dt).count();

  // Check aliasing
  if (rot_speed_sim * dt > M_PI)
    TOBAS_WARN_THROTTLE(kWarnPeriod, "Aliasing on motor [", channel_, "] might occur. Lower simulation time step.");

  // Update simulation state
  applyWrench(ecm, rot_speed_real, info.simTime);
  updateRotationSpeed(ecm, rot_speed_real, dt);
}

void GazeboRotorPlugin::registerROSInterfaces()
{
  const auto suffix = to_string(channel_);

  state_pub_ = createPublisher<tobas_msgs::msg::RotorState>(kRotorStateTopicPrefix + suffix);
  state_gt_pub_ = createPublisher<tobas_msgs::msg::RotorState>(kRotorStateGtTopicPrefix + suffix);
  debug_pub_ = createPublisher<tobas_gazebo_msgs::msg::RotorDebug>(kDebugTopicPrefix + suffix);

  throttle_sub_ = createSubscriber(kThrottleTopicPrefix + suffix, &self::throttleCmdCb, this);
  battery_gt_sub_ = createSubscriber(kBatteryGtTopic, &self::batteryGtCb, this);
  wind_gt_sub_ = createSubscriber(kWindGtTopic, &self::windSpeedGtCb, this);

  break_ss_ = createService<BreakSrv>(kBreakRotorSrvPrefix + suffix, &self::breakCb, this);
}

void GazeboRotorPlugin::addModelError()
{
  // 一様乱数を作成
  random_device rnd_dev;
  mt19937 rnd_gen(rnd_dev());
  UniformDistribution uniform(-1, 1);

  // モータ
  kv_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
  resistance_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));

  // プロペラ
  motor_const_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
  moment_const_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
  rotor_drag_coef_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
}

void GazeboRotorPlugin::applyWrench(
  sim::EntityComponentManager& ecm,
  double rot_speed,
  const steady_clock::duration& cur_time)
{
  assert(rot_speed >= 0.);

  // The True Role of Accelerometer Feedback in Quadrotor Control [Martin+, 2010]
  // II-A. Model of a single propeller near hovering
  // TODO: Implement other terms
  // TODO: II-B. Model of the complete quadrotor

  // Get joint axes
  const auto& local_axis = joint_->Axis(ecm).value().at(0).Xyz();
  const auto global_axis = link_->WorldPose(ecm).value().Rot().RotateVector(local_axis);

  // (1) first term: Thrust Force
  const auto thrust = motor_const_ * ::math::sqr(rot_speed);
  const auto thrust_W = thrust * global_axis;
  link_->AddWorldWrench(ecm, thrust_W, Vector3d::Zero);

  // (1) second term: H-force
  const auto linvel_W = link_->WorldLinearVelocity(ecm).value() - wind_vel_W_;
  const auto linvel_perp_W = linvel_W - (linvel_W.Dot(global_axis) * global_axis);
  const auto h_force_W = (-rot_speed * rotor_drag_coef_) * linvel_perp_W;
  link_->AddWorldWrench(ecm, h_force_W, Vector3d::Zero);

  // (2) first term: Rotor drag torque
  const auto torque = moment_const_ * thrust;
  const auto drag_torque_W = (-direction_ * torque) * global_axis;
  parent_link_->AddWorldWrench(ecm, Vector3d::Zero, drag_torque_W);

  // Compute electric current
  const auto kt = 1. / kv_;  // トルク定数 = 発電係数 = Kvの逆数 (内部抵抗値に依らない)
  const auto current = torque / kt;

  // 安全のため，一瞬でも過電流が流れたらESCが焼き切れたとみなす
  if (current > max_current_)
  {
    TOBAS_ERROR(
      "The ESC of rotor ", channel_, " is critically damaged due to an overcurrent of ", current,
      " A, which exceeded its maximum current capacity of ", max_current_, " A.");
    is_intact_ = false;
    throttle_ = 0.;
  }

  // Publish observed state
  // TODO: 観測ノイズを付加
  // TODO: 周波数を調整
  auto state_msg_obs = make_unique<tobas_msgs::msg::RotorState>();
  state_msg_obs->channel = channel_;
  if (is_intact_)
  {
    state_msg_obs->speed = rot_speed;
    state_msg_obs->current = current;
    state_msg_obs->status = tobas_msgs::msg::RotorState::ALL_FIELDS_READY;
  }
  else
  {
    state_msg_obs->speed = nan("ESC is broken.");
    state_msg_obs->current = nan("ESC is broken.");
    state_msg_obs->status = tobas_msgs::msg::RotorState::NO_COMMUNICATION;
  }
  state_pub_->publish(move(state_msg_obs));

  // Publish ground-truth state
  auto state_msg_gt = make_unique<tobas_msgs::msg::RotorState>();
  state_msg_gt->speed = rot_speed;
  state_msg_gt->current = current;
  state_msg_gt->status = tobas_msgs::msg::RotorState::ALL_FIELDS_READY;
  state_gt_pub_->publish(move(state_msg_gt));

  // Publish debug message
  auto debug_msg = make_unique<tobas_gazebo_msgs::msg::RotorDebug>();
  ros2::timeChronoToMsg(cur_time, debug_msg->header.stamp);
  debug_msg->rotation_speed = joint_->Velocity(ecm).value().at(0) * kRotorSpeedSlowdownSim;
  vectorGazeboToMsg(thrust_W, debug_msg->thrust_force);
  vectorGazeboToMsg(h_force_W, debug_msg->horizontal_force);
  vectorGazeboToMsg(drag_torque_W, debug_msg->drag_torque);
  debug_pub_->publish(move(debug_msg));
}

void GazeboRotorPlugin::updateRotationSpeed(sim::EntityComponentManager& ecm, double cur_speed, double dt)
{
  assert(cur_speed >= 0);
  assert(dt >= 0);

  // モータダイナミクスの係数 (memo: 2-78)
  const auto a = 2. * L_KV * moment_const_ * motor_const_;
  const auto b = resistance_ * kv_ * moment_const_ * motor_const_;
  const auto c = 1. / kv_;

  const auto Ea = battery_->voltage * throttle_;                                            // 印加電圧
  const auto eq_speed = Ea == 0. ? 0. : (sqrt(::math::sqr(c) + 4 * b * Ea) - c) / (2 * b);  // 平衡点での回転数

  // 次の時刻の回転数を求める
  double next_speed;
  if (cur_speed < 1e-3)
  {
    // モータの慣性モーメントを無視していることにより，現在の回転数が0のときは理論上ゼロ時間で平衡点に収束する．
    next_speed = eq_speed;
  }
  else
  {
    // 次のステップの回転数を計算
    const auto speed_rate = (Ea / cur_speed - b * cur_speed - c) / a;
    next_speed = cur_speed + speed_rate * dt;

    // 速度変化が大きすぎるなどして平衡点を飛び越えている場合は平衡点に拘束する
    if ((cur_speed - eq_speed) * (next_speed - eq_speed) < 0)
      next_speed = eq_speed;
  }

  // 次のステップの回転数をGazeboに反映
  joint_->SetVelocity(ecm, { direction_ * next_speed / kRotorSpeedSlowdownSim });
}

void GazeboRotorPlugin::throttleCmdCb(const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& msg)
{
  // バッテリーの情報が無いか電圧が低すぎたら応答なし
  if (battery_ == nullptr || battery_->voltage < kMinBatteryVoltage)
    return;

  // 壊れていたら応答なし
  if (!is_intact_)
    return;

  // 最後にコマンドを受け取った時刻を更新
  last_cmd_time_ = prev_sim_time_;

  // 範囲を制限してスロットルを更新
  if (msg->data < 0. || 1. < msg->data)
    TOBAS_ERROR("The commanded throttle ", msg->data, " is out of range.");
  throttle_ = std::clamp(msg->data, tobas::kMinThrot, tobas::kMaxThrot);
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

void GazeboRotorPlugin::breakCb(const BreakSrv::Request::ConstSharedPtr&, const BreakSrv::Response::SharedPtr& res)
{
  if (is_intact_)
  {
    is_intact_ = false;
    throttle_ = 0.;
    res->message = "Rotor " + to_string(channel_) + " has been broken.";
  }
  else
  {
    res->message = "Rotor " + to_string(channel_) + " is already broken.";
  }

  res->success = true;
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboRotorPlugin,
  sim::System,
  gazebo::GazeboRotorPlugin::ISystemConfigure,
  gazebo::GazeboRotorPlugin::ISystemPreUpdate)
