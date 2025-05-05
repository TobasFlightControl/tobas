#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_gazebo_common/constants.hpp>

#include <std_srvs/srv/empty.hpp>

#include <tobas_msgs/msg/battery.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>

#include "../include/tobas_gazebo_system_plugins/common/common.hpp"
#include "../include/tobas_gazebo_system_plugins/conversions/gazebo_msg.hpp"
#include "../include/tobas_gazebo_system_plugins/rate_manager.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
class GazeboBatteryPlugin : public BaseNode,
                            public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPostUpdate
{
  // Constants
  static constexpr double kSagCapRate = 0.2;  // [-] 放電特性が急激に変化する点における電気残率

  // Default parameters
  static constexpr double kDefaultVoltageNoiseStddev = 0.01;  // [V]
  static constexpr double kDefaultCurrentNoiseStddev = 0.01;  // [A]

  using self = GazeboBatteryPlugin;

public:
  explicit GazeboBatteryPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  size_t update_rate_;
  double max_voltage_;  // [V] 満充電時の電圧
  double sag_voltage_;  // [V] 放電特性が急激に変化する電圧．LiPoなら1セルあたり3.4Vくらい．
  double max_current_;  // [A] 最大電流
  double capacity_;     // [As] 電気容量
  double registance_;   // [Ω] 内部抵抗値
  double voltage_noise_stddev_;  // [V] 電圧の観測ノイズの標準偏差
  double current_noise_stddev_;  // [A] 電流の観測ノイズの標準偏差
  vector<string> rotor_link_names_;

  map<string, double> rotor_currents_;  // [A] 各モータに流れる電流
  double q_;                            // [As] 現在の電気量
  RateManager::SharedPtr rate_manager_;

  // Noise generator
  random_device rnd_dev_;
  mt19937 rnd_gen_;
  NormalDistribution voltage_noise_;
  NormalDistribution current_noise_;

  // Publishers
  ros2::PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::Battery> battery_gt_pub_;

  // Subscribers
  vector<ros2::SubscriberPtr<tobas_gazebo_msgs::msg::RotorState>> rotor_state_subs_;

  // Service servers
  ros2::ServiceServerPtr<std_srvs::srv::Empty> charge_srv_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerPubSub();
  double currentVoltage();

  void chargeCb(
    const std_srvs::srv::Empty::Request::ConstSharedPtr& req,
    const std_srvs::srv::Empty::Response::SharedPtr& res);
};

GazeboBatteryPlugin::GazeboBatteryPlugin() : rnd_gen_(rnd_dev_())
{
}

void GazeboBatteryPlugin::Configure(
  const gz::sim::Entity&,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager&,
  gz::sim::EventManager&)
{
  initialize("gazebo_battery_plugin", sdf);
  getSdfParams(sdf);

  q_ = capacity_;
  rate_manager_ = make_shared<RateManager>(update_rate_);

  voltage_noise_ = NormalDistribution(0., voltage_noise_stddev_);
  current_noise_ = NormalDistribution(0., current_noise_stddev_);

  registerPubSub();
  charge_srv_ = createService<std_srvs::srv::Empty>(kChargeBatterySrv, &self::chargeCb, this);
}

void GazeboBatteryPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "updateRate", update_rate_, NON_NEGATIVE);
  getSdfParam(sdf, "maxVoltage", max_voltage_, POSITIVE);
  getSdfParam(sdf, "sagVoltage", sag_voltage_, NON_NEGATIVE);
  getSdfParam(sdf, "maxCurrent", max_current_, POSITIVE);
  getSdfParam(sdf, "currentCapacity", capacity_, POSITIVE);
  getSdfParam(sdf, "internalRegistance", registance_, NON_NEGATIVE);
  getSdfParam(sdf, "voltageNoiseStddev", voltage_noise_stddev_, kDefaultVoltageNoiseStddev, NON_NEGATIVE);
  getSdfParam(sdf, "currentNoiseStddev", current_noise_stddev_, kDefaultCurrentNoiseStddev, NON_NEGATIVE);
  getSdfParam(sdf, "rotorLinkNames", rotor_link_names_);
}

void GazeboBatteryPlugin::registerPubSub()
{
  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(tobas::kBatteryTopic);
  battery_gt_pub_ = createPublisher<tobas_msgs::msg::Battery>(kBatteryGtTopic);

  // モータ状態のコールバックとサブスクライバを設定
  for (const auto& link_name : rotor_link_names_) {
    const auto topic = path::join(kRotorStateGtTopicNS, link_name);
    const auto qos = ros2::makeQoS(false, false, 1);
    const auto cb = [this, link_name](const tobas_gazebo_msgs::msg::RotorState::ConstSharedPtr& msg)
    {
      assert(msg->current >= 0.);
      rotor_currents_[link_name] = msg->current;
    };
    const auto sub = node_->create_subscription<tobas_gazebo_msgs::msg::RotorState>(topic, qos, cb);
    rotor_state_subs_.push_back(sub);
  }
}

void GazeboBatteryPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  if (rotor_currents_.size() < rotor_link_names_.size()) {
    if (info.simTime > kWarnStartTime) {
      const auto num_not_received = rotor_link_names_.size() - rotor_currents_.size();
      TOBAS_WARN_THROTTLE(kWarnPeriod, to_string(num_not_received), " rotor states are not received yet.");
    }
  }

  // 電流を計算
  double current_true = 0.;
  for (const auto& [_, current] : rotor_currents_) {
    current_true += current;
  }
  if (current_true > max_current_) {
    TOBAS_WARN_THROTTLE(kWarnPeriod, "The battery current is over limit: ", current_true, " > ", max_current_, " [A]");
  }
  const auto current_obs = max(current_true + current_noise_(rnd_gen_), 0.);  // 観測ノイズを受けた観測電流

  // 電気容量の減少
  const auto dt = chrono::duration<double>(info.dt).count();
  q_ = max(q_ - current_true * dt, 0.);

  // 電圧を計算
  const auto voltage_in = currentVoltage();                                   // 内部電圧
  const auto voltage_out = max(voltage_in - registance_ * current_true, 0.);  // 内部抵抗による電圧降下
  const auto voltage_obs = max(voltage_out + voltage_noise_(rnd_gen_), 0.);   // 観測ノイズを受けた観測電圧

  // 観測したバッテリーの状態を発行
  auto battery = make_unique<tobas_msgs::msg::Battery>();
  ros2::timeChronoToMsg(info.simTime, battery->header.stamp);
  battery->voltage = voltage_obs;
  battery->current = current_obs;
  battery_pub_->publish(move(battery));

  // 真のバッテリーの状態を発行
  auto battery_gt = make_unique<tobas_msgs::msg::Battery>();
  ros2::timeChronoToMsg(info.simTime, battery_gt->header.stamp);
  battery_gt->voltage = voltage_out;
  battery_gt->current = current_true;
  battery_gt_pub_->publish(move(battery_gt));
}

double GazeboBatteryPlugin::currentVoltage()
{
  // memo: 2-50
  const auto rate = q_ / capacity_;
  if (rate < kSagCapRate) {
    return sag_voltage_ * rate / kSagCapRate;
  }
  else {
    return (max_voltage_ - sag_voltage_) * (rate - kSagCapRate) / (1 - kSagCapRate) + sag_voltage_;
  }
}

void GazeboBatteryPlugin::chargeCb(
  const std_srvs::srv::Empty::Request::ConstSharedPtr&,
  const std_srvs::srv::Empty::Response::SharedPtr&)
{
  q_ = capacity_;
  TOBAS_INFO("Battery is charged.");
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboBatteryPlugin,
  gz::sim::System,
  gazebo::GazeboBatteryPlugin::ISystemConfigure,
  gazebo::GazeboBatteryPlugin::ISystemPostUpdate)
