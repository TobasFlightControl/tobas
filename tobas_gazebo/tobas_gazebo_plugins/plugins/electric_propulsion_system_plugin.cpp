#include <gz/sim/Model.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <std_srvs/srv/trigger.hpp>

#include <tobas_math/core.hpp>
#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rotor_state.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs_adapter/wind.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/sdf.hpp"

// モータのインダクタンスが不明なことが多いため，Kvとの積が概ね一定になることを利用する．
// ESCが電子制御で電流の変化を抑えるため，見かけのインダクタンスは実測値よりも遥かに大きい (100倍くらい？)
// TODO: 実機の時定数がSIMよりも大きくならないように想定しうる最大値に設定する．時定数を直接設定するほうが現実的かも．
#define L_KV 10.

using namespace std;
using namespace chrono;
namespace cmp = gz::sim::components;

namespace gazebo
{
/* Simulates ESC, rotor and propeller. */
class GazeboElectricPropulsionSystemPlugin : public BaseNode,
                                             public gz::sim::System,
                                             public gz::sim::ISystemConfigure,
                                             public gz::sim::ISystemPreUpdate
{
  // Constants
  static constexpr double kAutoStopTimeout = 0.5;    // [s]
  static constexpr double kMinBatteryVoltage = 3.;   // [V]
  static constexpr double kThrotLimitMargin = 1e-3;  // [-]

  // Default parameters
  static constexpr size_t kDefaultPublishStateRate = 400;  // [Hz]
  static constexpr double kDefaultRotorNoiseCoef = 0.5;    // [-]
  static constexpr double kDefaultMaxModelErrorRate = 0.;

  using self = GazeboElectricPropulsionSystemPlugin;
  using BreakSrv = std_srvs::srv::Trigger;

public:
  explicit GazeboElectricPropulsionSystemPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  string link_name_;
  double kv_;                    // [rad/s/V]
  double resistance_;            // [Ω]
  size_t num_blades_;            // [-]
  double motor_const_;           // [N/(rad/s)^2]
  double moment_const_;          // [m]
  double drag_const_;            // [N*s^2/rad/m]
  int direction_;                // Turning direction: 1(CCW) or -1(CW)
  double max_current_;           // [A] ESCの最大電流
  size_t publish_state_rate_;    // [Hz]
  double noise_coef_;            // [-]
  double max_model_error_rate_;  // [-]

  double throttle_ = 0.;  // [0, 1]
  double velocity_ = 0.;  // [rad/s]
  double position_ = 0.;  // [rad]
  tobas_msgs::msg::Battery::ConstSharedPtr battery_gt_;
  gz::math::Vector3d wind_vel_W_ = gz::math::Vector3d::Zero;  // [m/s]
  steady_clock::duration prev_sim_time_;
  steady_clock::duration last_cmd_time_;  // 最後にスロットルコマンドが指令された時刻
  bool is_intact_ = true;
  RateManager::SharedPtr publish_state_rate_manager_;

  // Gazebo objects
  shared_ptr<gz::sim::Joint> joint_;
  shared_ptr<gz::sim::Link> link_;
  shared_ptr<gz::sim::Link> parent_link_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorState> state_pub_;
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::RotorState> state_gt_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::Throttle> throttle_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_gt_sub_;
  ros2::SubscriberPtr<tobas_msgs::Wind> wind_gt_sub_;

  // Services
  ros2::ServiceServerPtr<BreakSrv> break_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerROSInterfaces();
  void addModelError();

  double velocitySim() const;
  void applyWrenchAndPublishState(gz::sim::EntityComponentManager& ecm, const steady_clock::duration& cur_time);
  void updateJointState(gz::sim::EntityComponentManager& ecm, double dt);

  void throttleCmdCb(const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& throttle);
  void batteryGtCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery_gt);
  void windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind_gt);

  void breakCb(const BreakSrv::Request::ConstSharedPtr& req, const BreakSrv::Response::SharedPtr& res);
};

GazeboElectricPropulsionSystemPlugin::GazeboElectricPropulsionSystemPlugin()
{
}

void GazeboElectricPropulsionSystemPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_electric_propulsion_system_plugin", sdf);
  getSdfParams(sdf);

  publish_state_rate_manager_ = make_shared<RateManager>(publish_state_rate_);
  addModelError();

  // Get robot model
  gz::sim::Model model(model_entity);
  if (!model.Valid(ecm))
    TOBAS_EXIT("Failed to find model.");

  // Get joint
  const auto joint_entity = findJointWithChildLink(ecm, link_name_);
  if (!joint_entity.has_value())
    TOBAS_EXIT("Failed to find the parent joint of rotor link \"", link_name_, "\".");
  joint_ = make_shared<gz::sim::Joint>(joint_entity.value());
  if (!joint_->Valid(ecm))
    TOBAS_EXIT("Failed to find rotor link \"", link_name_, "\".");

  // Get joint name
  const auto joint_name = joint_->Name(ecm).value();

  // Check joint type
  const auto joint_type = joint_->Type(ecm).value();
  if (joint_type != sdf::JointType::CONTINUOUS && joint_type != sdf::JointType::REVOLUTE)
    TOBAS_EXIT("Joint \"", joint_name, "\" is not a rotating joint.");

  // Get child link
  const auto link_entity = model.LinkByName(ecm, link_name_);
  link_ = make_shared<gz::sim::Link>(link_entity);
  if (!link_->Valid(ecm))
    TOBAS_EXIT("Failed to find the child link \"", link_name_, "\".");

  // Get parent link
  const auto parent_link_name = joint_->ChildLinkName(ecm).value();
  const auto parent_link_entity = model.LinkByName(ecm, parent_link_name);
  parent_link_ = make_shared<gz::sim::Link>(parent_link_entity);
  if (!parent_link_->Valid(ecm))
    TOBAS_EXIT("Failed to find the parent link \"", parent_link_name, "\".");

  // Create necessary components
  if (!getComponent<cmp::JointAxis>(joint_entity.value(), ecm))
    TOBAS_EXIT("Failed to get component JointAxis of joint \"", joint_name, "\".");
  if (!getComponent<cmp::JointVelocity>(joint_entity.value(), ecm))
    TOBAS_EXIT("Failed to get component JointVelocity of joint \"", joint_name, "\".");
  if (!getComponent<cmp::WorldPose>(link_entity, ecm))
    TOBAS_EXIT("Failed to get component WorldPose of link \"", link_name_, "\".");
  if (!getComponent<cmp::WorldLinearVelocity>(link_entity, ecm))
    TOBAS_EXIT("Failed to get component WorldLinearVelocity of link \"", link_name_, "\".");

  // Register ROS interfaces
  registerROSInterfaces();
}

void GazeboElectricPropulsionSystemPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);

  getSdfParam(sdf, "kv", kv_, POSITIVE);
  getSdfParam(sdf, "internalResistance", resistance_, POSITIVE);
  getSdfParam(sdf, "numberOfBlades", num_blades_, POSITIVE);

  getSdfParam(sdf, "motorConstant", motor_const_, POSITIVE);
  getSdfParam(sdf, "momentConstant", moment_const_, POSITIVE);
  getSdfParam(sdf, "dragConstant", drag_const_, NON_NEGATIVE);

  if (!getTurningDirection(sdf, direction_))
    TOBAS_EXIT("Failed to get turning direction.");

  getSdfParam(sdf, "maxCurrent", max_current_, POSITIVE);

  getSdfParam(sdf, "publishStateRate", publish_state_rate_, kDefaultPublishStateRate, NON_NEGATIVE);
  getSdfParam(sdf, "rotorNoiseCoefficient", noise_coef_, kDefaultRotorNoiseCoef, NON_NEGATIVE);
  getSdfParam(sdf, "maxModelErrorRate", max_model_error_rate_, kDefaultMaxModelErrorRate, NON_NEGATIVE);
}

void GazeboElectricPropulsionSystemPlugin::PreUpdate(
  const gz::sim::UpdateInfo& info,
  gz::sim::EntityComponentManager& ecm)
{
  // Update the previous simulation step time
  prev_sim_time_ = info.simTime;

  // Check topics
  if (battery_gt_ == nullptr)
  {
    if (info.simTime > kWarnStartTime)
      TOBAS_WARN_THROTTLE(kWarnPeriod, "Battery message is not received yet.");
    return;
  }

  // 最後にスロットルコマンドが指令された時刻から一定時間経過したら強制的にモータを停止する
  const auto secs_from_last_cmd = duration<double>(info.simTime - last_cmd_time_).count();
  if (secs_from_last_cmd > kAutoStopTimeout)
    throttle_ = 0.;

  // Compute time after previous simulation time
  const auto dt = duration<double>(info.dt).count();

  // Check aliasing
  if (fabs(velocitySim() * dt) > M_PI)
    TOBAS_WARN_THROTTLE(kWarnPeriod, "Aliasing on motor \"", link_name_, "\" might occur. Lower simulation time step.");

  // Update simulation state
  applyWrenchAndPublishState(ecm, info.simTime);
  updateJointState(ecm, dt);
}

void GazeboElectricPropulsionSystemPlugin::registerROSInterfaces()
{
  state_pub_ = createPublisher<tobas_msgs::msg::RotorState>(path::join(kRotorStateTopicNS, link_name_));
  state_gt_pub_ = createPublisher<tobas_gazebo_msgs::msg::RotorState>(path::join(kRotorStateGtTopicNS, link_name_));

  throttle_sub_ = createSubscriber(path::join(kRotorThrottleCmdTopicNS, link_name_), &self::throttleCmdCb, this);
  battery_gt_sub_ = createSubscriber(kBatteryGtTopic, &self::batteryGtCb, this);
  wind_gt_sub_ = createSubscriber(kWindGtTopic, &self::windSpeedGtCb, this);

  break_ss_ = createService<BreakSrv>(path::join(kBreakRotorSrvNS, link_name_), &self::breakCb, this);
}

void GazeboElectricPropulsionSystemPlugin::addModelError()
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
  drag_const_ *= (1 + max_model_error_rate_ * uniform(rnd_gen));
}

double GazeboElectricPropulsionSystemPlugin::velocitySim() const
{
  return velocity_ / kRotorSpeedSlowdownSim;
}

void GazeboElectricPropulsionSystemPlugin::applyWrenchAndPublishState(
  gz::sim::EntityComponentManager& ecm,
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
  const auto thrust = motor_const_ * math::sqr(velocity_);
  const auto thrust_W = thrust * global_axis;
  link_->AddWorldWrench(ecm, thrust_W, gz::math::Vector3d::Zero);

  // (1) second term: H-force
  const auto linvel_W = link_->WorldLinearVelocity(ecm).value() - wind_vel_W_;
  const auto linvel_perp_W = linvel_W - (linvel_W.Dot(global_axis) * global_axis);
  const auto h_force_W = (-fabs(velocity_) * drag_const_) * linvel_perp_W;
  link_->AddWorldWrench(ecm, h_force_W, gz::math::Vector3d::Zero);

  // (2) first term: Rotor drag torque
  const auto torque = moment_const_ * thrust;
  const auto drag_torque_W = (-direction_ * torque) * global_axis;
  parent_link_->AddWorldWrench(ecm, gz::math::Vector3d::Zero, drag_torque_W);

  // Compute electric current
  const auto kt = 1. / kv_;  // トルク定数 = 発電係数 = Kvの逆数 (内部抵抗値に依らない)
  const auto current = torque / kt;

  // 安全のため，一瞬でも過電流が流れたらESCが焼き切れたとみなす
  if (current > max_current_)
  {
    TOBAS_ERROR(
      "The ESC of rotor \"", link_name_, "\" is critically damaged due to an overcurrent of ", current,
      " A, which exceeded its maximum current capacity of ", max_current_, " A.");
    is_intact_ = false;
    throttle_ = 0.;
  }

  // Publish observed state
  if (publish_state_rate_manager_->update(cur_time))
  {
    auto state_msg_obs = make_unique<tobas_msgs::msg::RotorState>();
    state_msg_obs->link_name = link_name_;
    if (is_intact_)
    {
      state_msg_obs->speed = direction_ * velocity_;
      state_msg_obs->thrust = thrust;
      state_msg_obs->status = tobas_msgs::msg::RotorState::NO_ERROR;
    }
    else
    {
      state_msg_obs->speed = NAN;
      state_msg_obs->thrust = NAN;
      state_msg_obs->status = tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE;
    }
    state_pub_->publish(move(state_msg_obs));
  }

  // Publish ground-truth state
  auto state_msg_gt = make_unique<tobas_gazebo_msgs::msg::RotorState>();
  ros2::timeChronoToMsg(cur_time, state_msg_gt->header.stamp);
  state_msg_gt->rotation_speed = direction_ * velocity_;
  state_msg_gt->current = current;
  state_msg_gt->rotor_noise = noise_coef_ * thrust * sin(num_blades_ * position_);  // TODO: 倍周波も考慮
  state_gt_pub_->publish(move(state_msg_gt));
}

void GazeboElectricPropulsionSystemPlugin::updateJointState(gz::sim::EntityComponentManager& ecm, double dt)
{
  // モータダイナミクスの係数 (memo: 2-78)
  const auto a = 2. * L_KV * moment_const_ * motor_const_;
  const auto b = resistance_ * kv_ * moment_const_ * motor_const_;
  const auto c = 1. / kv_;

  const auto Ea = battery_gt_->voltage * throttle_;                                         // 印加電圧
  const auto eq_speed = Ea == 0. ? 0. : (sqrt(::math::sqr(c) + 4 * b * Ea) - c) / (2 * b);  // 平衡点での回転数

  const auto cur_speed = max(direction_ * velocity_, 0.);

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

  // ジョイントの状態を更新
  position_ += velocity_ * dt;
  velocity_ = direction_ * next_speed;

  // 視認用にGazeboに反映
  joint_->SetVelocity(ecm, { velocitySim() });
}

void GazeboElectricPropulsionSystemPlugin::throttleCmdCb(
  const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& throttle)
{
  // バッテリーの情報が無いか電圧が低すぎたら応答なし
  if (battery_gt_ == nullptr || battery_gt_->voltage < kMinBatteryVoltage)
    return;

  // 壊れていたら応答なし
  if (!is_intact_)
    return;

  // 最後にコマンドを受け取った時刻を更新
  last_cmd_time_ = prev_sim_time_;

  // 範囲を制限してスロットルを更新
  if (throttle->data < tobas::kMinThrot - kThrotLimitMargin || tobas::kMaxThrot + kThrotLimitMargin < throttle->data)
    TOBAS_ERROR("The commanded throttle ", throttle->data, " is out of range.");
  throttle_ = std::clamp(throttle->data, tobas::kMinThrot, tobas::kMaxThrot);
}

void GazeboElectricPropulsionSystemPlugin::batteryGtCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery_gt)
{
  battery_gt_ = battery_gt;
}

void GazeboElectricPropulsionSystemPlugin::windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind_gt)
{
  vectorKDLToGazebo(wind_gt->vel, wind_vel_W_);
}

void GazeboElectricPropulsionSystemPlugin::breakCb(
  const BreakSrv::Request::ConstSharedPtr&,
  const BreakSrv::Response::SharedPtr& res)
{
  if (is_intact_)
  {
    is_intact_ = false;
    throttle_ = 0.;
    res->message = "Rotor \"" + link_name_ + "\" has been broken.";
  }
  else
  {
    res->message = "Rotor \"" + link_name_ + "\" is already broken.";
  }

  res->success = true;
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboElectricPropulsionSystemPlugin,
  gz::sim::System,
  gazebo::GazeboElectricPropulsionSystemPlugin::ISystemConfigure,
  gazebo::GazeboElectricPropulsionSystemPlugin::ISystemPreUpdate)
