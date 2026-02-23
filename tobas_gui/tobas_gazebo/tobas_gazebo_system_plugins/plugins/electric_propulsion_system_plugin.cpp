#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>

#include <tobas_constants/throttle.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_kdl.hpp>
#include <tobas_gazebo_conversions/gazebo_ros.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_math/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tobas_gazebo_msgs/msg/rotor_debug.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>
#include <tobas_gazebo_msgs/msg/throttle.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_state.hpp>
#include <tobas_msgs_adapter/wind.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/node_name.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"

// モータのインダクタンスが不明なことが多いため，Kvとの積が概ね一定になることを利用する．
// ESCが電子制御で電流の変化を抑えるため，見かけのインダクタンスは実測値よりも遥かに大きい (100倍くらい？)
// TODO: 実機の時定数がSIMよりも大きくならないように想定しうる最大値に設定する．時定数を直接設定するほうが現実的かも．
#define L_KV 2.

namespace ch = std::chrono;
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
  static constexpr char kDebugTopicNS[] = "gazebo/rotor_debug";
  static constexpr double kAutoStopTimeout = 0.5;    // [s]
  static constexpr double kMinBatteryVoltage = 3.;   // [V]
  static constexpr double kThrotLimitMargin = 1e-3;  // [-]

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
  std::string link_name_;
  struct Param
  {
    double kv;                  // [rad/s/V]
    double resistance;          // [Ω]
    size_t num_blades;          // [-]
    double motor_const;         // [N/(rad/s)^2]
    double moment_const;        // [m]
    double drag_const;          // [N*s^2/rad/m]
    int direction;              // Turning direction: 1(CCW) or -1(CW)
    double max_current;         // [A] ESCの最大電流
    size_t publish_state_rate;  // [Hz]
    double vib_force_coef;      // [-]
    double vib_force_var_rate;  // [-]
  } param_;

  double throt_ = 0.;  // [0, 1]
  double acc_ = 0.;    // [rad/s^2]
  double vel_ = 0.;    // [rad/s]
  double pos_ = 0.;    // [rad]
  tobas_msgs::msg::Battery::ConstSharedPtr battery_gt_;
  gz::math::Vector3d wind_vel_W_ = gz::math::Vector3d::Zero;  // [m/s]
  ch::steady_clock::duration prev_sim_time_;
  ch::steady_clock::duration last_cmd_time_;  // 最後にスロットルコマンドが指令された時刻
  bool is_intact_ = true;
  RateManager::SharedPtr publish_state_rate_manager_;

  // Random
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  RiceDistribution rice_;

  // Gazebo objects
  std::shared_ptr<gz::sim::Joint> joint_;
  std::shared_ptr<gz::sim::Link> link_;
  std::shared_ptr<gz::sim::Link> parent_link_;
  const cmp::JointAxis* jnt_axis_;
  const cmp::JointVelocity* jnt_vel_;
  const cmp::WorldPose* pose_W_;
  const cmp::WorldLinearVelocity* linvel_W_;
  const cmp::WorldAngularVelocity* angvel_W_;
  const cmp::Inertial* inertial_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::RotorState> state_pub_;
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::RotorState> state_gt_pub_;
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::RotorDebug> debug_pub_;

  // Subscribers
  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::Throttle> throttle_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Battery> battery_gt_sub_;
  ros2::SubscriberPtr<tobas_msgs::Wind> wind_gt_sub_;

  // Services
  ros2::ServiceServerPtr<BreakSrv> break_ss_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerRosInterfaces();

  double velocitySim() const;

  /**
   * @brief Gazeboには含まれない力を加える．
   *
   * - 慣性力とコリオリ力: Gazebo上では低速回転になっているため，回転体の運動により発生するモーメントを別で与える必要がある．
   *
   * - 大気から受ける空気力: 推力，反トルク，回転軸と垂直の方向に発生する抗力．
   */
  void applyWrenchAndPublishState(gz::sim::EntityComponentManager& ecm, const ch::steady_clock::duration& cur_time);

  void updateJointState(gz::sim::EntityComponentManager& ecm, double dt);

  void throttleCmdCb(const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& throttle);
  void batteryGtCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery_gt);
  void windSpeedGtCb(const tobas_msgs::Wind::ConstSharedPtr& wind_gt);

  void breakCb(const BreakSrv::Request::ConstSharedPtr& req, const BreakSrv::Response::SharedPtr& res);
};

GazeboElectricPropulsionSystemPlugin::GazeboElectricPropulsionSystemPlugin() : rnd_gen_(rnd_dev_())
{
}

void GazeboElectricPropulsionSystemPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  link_name_ = sdf->Get<std::string>("linkName");
  initialize("gazebo_" + sanitizeNodeName(link_name_) + "_controller_plugin", sdf);
  getSdfParams(sdf);

  rice_ = RiceDistribution(1., param_.vib_force_var_rate);

  publish_state_rate_manager_ = std::make_shared<RateManager>(param_.publish_state_rate);

  // Get robot model
  const gz::sim::Model model(model_entity);
  if (!model.Valid(ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Get joint
  const auto joint_entity = findJointWithChildLink(ecm, link_name_);
  if (!joint_entity.has_value()) {
    TOBAS_EXIT("Failed to find the parent joint of rotor link \"", link_name_, "\".");
  }
  joint_ = std::make_shared<gz::sim::Joint>(joint_entity.value());
  if (!joint_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find rotor link \"", link_name_, "\".");
  }

  // Get joint name
  const auto joint_name = joint_->Name(ecm).value();

  // Check joint type
  const auto joint_type = joint_->Type(ecm).value();
  if (joint_type != sdf::JointType::CONTINUOUS && joint_type != sdf::JointType::REVOLUTE) {
    TOBAS_EXIT("Joint \"", joint_name, "\" is not a rotating joint.");
  }

  // Get child link
  const auto link_entity = model.LinkByName(ecm, link_name_);
  link_ = std::make_shared<gz::sim::Link>(link_entity);
  if (!link_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find the child link \"", link_name_, "\".");
  }

  // Get parent link
  const auto parent_link_name = joint_->ParentLinkName(ecm).value();
  const auto parent_link_entity = model.LinkByName(ecm, parent_link_name);
  parent_link_ = std::make_shared<gz::sim::Link>(parent_link_entity);
  if (!parent_link_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find the parent link \"", parent_link_name, "\".");
  }

  // Create necessary components
  TOBAS_CHECK(jnt_axis_ = getComponent<cmp::JointAxis>(joint_entity.value(), ecm));
  TOBAS_CHECK(jnt_vel_ = getComponent<cmp::JointVelocity>(joint_entity.value(), ecm));
  TOBAS_CHECK(pose_W_ = getComponent<cmp::WorldPose>(link_entity, ecm));
  TOBAS_CHECK(linvel_W_ = getComponent<cmp::WorldLinearVelocity>(link_entity, ecm));
  TOBAS_CHECK(angvel_W_ = getComponent<cmp::WorldAngularVelocity>(link_entity, ecm));
  TOBAS_CHECK(inertial_ = getComponent<cmp::Inertial>(link_entity, ecm));

  // Register ROS interfaces
  registerRosInterfaces();
}

void GazeboElectricPropulsionSystemPlugin::PreUpdate(
  const gz::sim::UpdateInfo& info,
  gz::sim::EntityComponentManager& ecm)
{
  // Update the previous simulation step time
  prev_sim_time_ = info.simTime;

  // Check topics
  if (!battery_gt_) {
    if (info.simTime > kCheckTopicWarnStartTime) {
      TOBAS_WARN_THROTTLE(kWarnPeriod, "Battery message is not received yet.");
    }
    return;
  }

  // 最後にスロットルコマンドが指令された時刻から一定時間経過したら強制的にモータを停止する
  const auto secs_from_last_cmd = ch::duration<double>(info.simTime - last_cmd_time_).count();
  if (secs_from_last_cmd > kAutoStopTimeout) {
    throt_ = 0.;
  }

  // Compute time after previous simulation time
  const auto dt = ch::duration<double>(info.dt).count();

  // Check aliasing
  if (std::abs(velocitySim() * dt) > M_PI) {
    TOBAS_WARN_THROTTLE(kWarnPeriod, "Aliasing on motor \"", link_name_, "\" might occur. Lower simulation time step.");
  }

  // Update simulation state
  applyWrenchAndPublishState(ecm, info.simTime);
  updateJointState(ecm, dt);
}

void GazeboElectricPropulsionSystemPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "kv", param_.kv, kPositive);
  getSdfParam(sdf, "internalResistance", param_.resistance, kPositive);
  getSdfParam(sdf, "numberOfBlades", param_.num_blades, kPositive);

  getSdfParam(sdf, "motorConstant", param_.motor_const, kPositive);
  getSdfParam(sdf, "momentConstant", param_.moment_const, kPositive);
  getSdfParam(sdf, "dragConstant", param_.drag_const, kNonNegative);

  if (!getTurningDirection(sdf, param_.direction)) {
    TOBAS_EXIT("Failed to get turning direction.");
  }

  getSdfParam(sdf, "maxCurrent", param_.max_current, kPositive);

  getSdfParam(sdf, "publishStateRate", param_.publish_state_rate, 400UL, kNonNegative);
  getSdfParam(sdf, "vibrationForceCoefficient", param_.vib_force_coef, 1.5, kNonNegative);
  getSdfParam(sdf, "vibrationForceVariationRate", param_.vib_force_var_rate, 0.3, kNonNegative);
}

void GazeboElectricPropulsionSystemPlugin::registerRosInterfaces()
{
  state_pub_ = createPublisher<tobas_msgs::msg::RotorState>(path::join(kRotorStateTopicNS, link_name_));
  state_gt_pub_ = createPublisher<tobas_gazebo_msgs::msg::RotorState>(path::join(kRotorStateGtTopicNS, link_name_));
  debug_pub_ = createPublisher<tobas_gazebo_msgs::msg::RotorDebug>(path::join(kDebugTopicNS, link_name_));

  throttle_sub_ = createSubscriber(path::join(kRotorThrottleCmdTopicNS, link_name_), &self::throttleCmdCb, this);
  battery_gt_sub_ = createSubscriber(kBatteryGtTopic, &self::batteryGtCb, this);
  wind_gt_sub_ = createSubscriber(kWindGtTopic, &self::windSpeedGtCb, this);

  break_ss_ = createService<BreakSrv>(path::join(kBreakRotorSrvNS, link_name_), &self::breakCb, this);
}

double GazeboElectricPropulsionSystemPlugin::velocitySim() const
{
  return vel_ / kRotorSpeedSlowdownSim;
}

void GazeboElectricPropulsionSystemPlugin::applyWrenchAndPublishState(
  gz::sim::EntityComponentManager& ecm,
  const ch::steady_clock::duration& cur_time)
{
  // Get joint axes
  const auto& R_W_L = pose_W_->Data().Rot();
  const auto& axis_L = jnt_axis_->Data().Xyz();
  const auto axis_W = R_W_L.RotateVector(axis_L);

  // Inertial moment
  const auto I_W = link_->WorldInertiaMatrix(ecm).value();  // 回転軸上に重心がある想定
  const auto inertial_moment_W = -(I_W * (acc_ * axis_W));

  // Coriolis moment (Gyro effect)
  const auto L_W = I_W * (vel_ * axis_W);  // プロペラの角運動量
  const auto coriolis_moment_W = -angvel_W_->Data().Cross(L_W);

  // External force: Thrust force
  const auto thrust = param_.motor_const * math::sqr(vel_);
  const auto thrust_force_W = thrust * axis_W;

  // External force: H-force
  const auto linvel_rel_W = linvel_W_->Data() - wind_vel_W_;
  const auto linvel_perp_W = linvel_rel_W - (linvel_rel_W.Dot(axis_W) * axis_W);
  const auto h_force_W = (-std::abs(vel_) * param_.drag_const) * linvel_perp_W;

  // External moment: Drag torque
  const auto torque = param_.moment_const * thrust;
  const auto drag_moment_W = (-param_.direction * torque) * axis_W;

  // Apply wrench
  link_->AddWorldWrench(ecm, thrust_force_W + h_force_W, gz::math::Vector3d::Zero);
  parent_link_->AddWorldWrench(ecm, gz::math::Vector3d::Zero, inertial_moment_W + coriolis_moment_W + drag_moment_W);

  // Compute electric current
  const auto kt = 1. / param_.kv;  // トルク定数 = 発電係数 = Kvの逆数 (内部抵抗値に依らない)
  const auto current = torque / kt;

  // 安全のため，一瞬でも過電流が流れたらESCが焼き切れたとみなす
  if (current > param_.max_current) {
    TOBAS_ERROR(
      "The ESC of rotor \"",
      link_name_,
      "\" is critically damaged due to an overcurrent of ",
      current,
      " A, which exceeded its maximum current capacity of ",
      param_.max_current,
      " A.");
    is_intact_ = false;
    throt_ = 0.;
  }

  // Publish observed state
  if (publish_state_rate_manager_->update(cur_time)) {
    auto state_msg_obs = std::make_unique<tobas_msgs::msg::RotorState>();
    state_msg_obs->link_name = link_name_;
    if (is_intact_) {
      state_msg_obs->speed = param_.direction * vel_;
      state_msg_obs->thrust = thrust;
      state_msg_obs->status = tobas_msgs::msg::RotorState::NO_ERROR;
    }
    else {
      state_msg_obs->speed = NAN;
      state_msg_obs->thrust = NAN;
      state_msg_obs->status = tobas_msgs::msg::RotorState::COMMUNICATION_FAILURE;
    }
    state_pub_->publish(std::move(state_msg_obs));
  }

  // Publish ground-truth state
  // TODO: 実機のIMUの周波数解析結果を分析してより正確な振動モデルを構築 (倍周波も考慮)
  auto state_msg_gt = std::make_unique<tobas_gazebo_msgs::msg::RotorState>();
  ros2::timeChronoToMsg(cur_time, state_msg_gt->header.stamp);
  state_msg_gt->rotation_speed = param_.direction * vel_;
  state_msg_gt->current = current;
  state_msg_gt->vibration_force = param_.vib_force_coef * thrust * sin(pos_) * rice_(rnd_gen_);
  state_gt_pub_->publish(std::move(state_msg_gt));

  // Publish debug information
  auto debug_msg = std::make_unique<tobas_gazebo_msgs::msg::RotorDebug>();
  ros2::timeChronoToMsg(cur_time, debug_msg->header.stamp);
  debug_msg->position = pos_;
  debug_msg->velocity = vel_;
  debug_msg->acceleration = acc_;
  vectorGazeboToRos(inertial_moment_W, debug_msg->inertia_moment);
  vectorGazeboToRos(coriolis_moment_W, debug_msg->coriolis_moment);
  vectorGazeboToRos(thrust_force_W, debug_msg->thrust_force);
  vectorGazeboToRos(h_force_W, debug_msg->h_force);
  vectorGazeboToRos(drag_moment_W, debug_msg->drag_momeent);
  debug_pub_->publish(std::move(debug_msg));
}

void GazeboElectricPropulsionSystemPlugin::updateJointState(gz::sim::EntityComponentManager& ecm, double dt)
{
  // モータダイナミクスの係数 (memo: 2-78)
  const auto a = 2. * L_KV * param_.moment_const * param_.motor_const;
  const auto b = param_.resistance * param_.kv * param_.moment_const * param_.motor_const;
  const auto c = 1. / param_.kv;

  const auto Ea = battery_gt_->voltage * throt_;                                            // 印加電圧
  const auto eq_speed = Ea == 0. ? 0. : (sqrt(::math::sqr(c) + 4 * b * Ea) - c) / (2 * b);  // 平衡点での回転数

  const auto cur_speed = std::max(param_.direction * vel_, 0.);

  // 次の時刻の回転数を求める
  double next_speed;
  if (cur_speed < 1e-3) {
    // モータの慣性モーメントを無視していることにより，現在の回転数が0のときは理論上ゼロ時間で平衡点に収束する．
    next_speed = eq_speed;
  }
  else {
    // 次のステップの回転数を計算
    const auto speed_rate = (Ea / cur_speed - b * cur_speed - c) / a;
    next_speed = cur_speed + speed_rate * dt;

    // 速度変化が大きすぎるなどして平衡点を飛び越えている場合は平衡点に拘束する
    if ((cur_speed - eq_speed) * (next_speed - eq_speed) < 0) {
      next_speed = eq_speed;
    }
  }

  // ジョイントの状態を更新
  const auto next_vel = param_.direction * next_speed;
  acc_ = (next_vel - vel_) / dt;  // 数値微分で加速度を計算
  pos_ += vel_ * dt;              // 前回の速度で積分するのが大事
  vel_ = next_vel;

  // 視認用にGazeboに反映
  joint_->SetVelocity(ecm, { velocitySim() });
}

void GazeboElectricPropulsionSystemPlugin::throttleCmdCb(const tobas_gazebo_msgs::msg::Throttle::ConstSharedPtr& throttle)
{
  // バッテリーの情報が無いか電圧が低すぎたら応答なし
  if (!battery_gt_ || battery_gt_->voltage < kMinBatteryVoltage) {
    return;
  }

  // 壊れていたら応答なし
  if (!is_intact_) {
    return;
  }

  // 最後にコマンドを受け取った時刻を更新
  last_cmd_time_ = prev_sim_time_;

  // 範囲を制限してスロットルを更新
  if (throttle->data < tobas::kMinThrot - kThrotLimitMargin || tobas::kMaxThrot + kThrotLimitMargin < throttle->data) {
    TOBAS_ERROR("The commanded throttle ", throttle->data, " is out of range.");
  }
  throt_ = std::clamp(throttle->data, tobas::kMinThrot, tobas::kMaxThrot);
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
  if (is_intact_) {
    is_intact_ = false;
    throt_ = 0.;
    res->message = "Rotor \"" + link_name_ + "\" has been broken.";
  }
  else {
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
