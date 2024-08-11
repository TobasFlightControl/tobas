#include <std_srvs/srv/empty.hpp>

#include <tobas_std_tools/vector.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/rotor_state.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"

using namespace std;
using namespace gz;
namespace cmp = sim::components;

namespace gazebo
{
class GazeboBatteryPlugin : public BaseNode,
                            public sim::System,
                            public sim::ISystemConfigure,
                            public sim::ISystemPostUpdate
{
  // Constants
  static constexpr double kSagCapRate = 0.2;  // [-] 放電特性が急激に変化する点における電気残率

  // Default parameters
  static constexpr size_t kDefaultUpdateRate = 100;          // [Hz]
  static constexpr double kDefaultVoltageNoiseStddev = 0.1;  // [V]
  static constexpr double kDefaultCurrentNoiseStddev = 0.;   // [A]

  using self = GazeboBatteryPlugin;

public:
  explicit GazeboBatteryPlugin();

  void Configure(
    const sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    sim::EntityComponentManager& ecm,
    sim::EventManager&) override;

  void PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager& ecm) override;

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
  size_t num_rotors_;

  vector<double> currents_;  // [A] 各モータに流れる電流
  double q_;                 // [As] 現在の電気量
  RateManager::SharedPtr rate_manager_;

  // Noise generator
  random_device rnd_dev_;
  mt19937 rnd_gen_;
  NormalDistribution voltage_noise_;
  NormalDistribution current_noise_;

  // Publishers
  PublisherPtr<tobas_msgs::msg::Battery> battery_pub_;
  PublisherPtr<tobas_msgs::msg::Battery> battery_gt_pub_;

  // Subscribers
  vector<SubscriberPtr<tobas_msgs::msg::RotorState>> rotor_state_subs_;

  // Service servers
  ServicePtr<std_srvs::srv::Empty> charge_srv_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerPubSub();
  double currentVoltage();

  void chargeCb(
    const std_srvs::srv::Empty::Request::ConstSharedPtr& req,
    const std_srvs::srv::Empty::Response::SharedPtr& res);
};

GazeboBatteryPlugin::GazeboBatteryPlugin() : BaseNode("battery_plugin"), rnd_gen_(rnd_dev_())
{
}

void GazeboBatteryPlugin::Configure(
  const sim::Entity&,
  const sdf::ElementConstPtr& sdf,
  sim::EntityComponentManager&,
  sim::EventManager&)
{
  initialize(sdf);
  getSdfParams(sdf);

  currents_.resize(num_rotors_, 0.);
  q_ = capacity_;
  rate_manager_ = make_shared<RateManager>(update_rate_);

  voltage_noise_ = NormalDistribution(0., voltage_noise_stddev_);
  current_noise_ = NormalDistribution(0., current_noise_stddev_);

  registerPubSub();
  charge_srv_ = createService<std_srvs::srv::Empty>(path::join(ns(), kChargeBatterySrv), &self::chargeCb, this);
}

void GazeboBatteryPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, POSITIVE);
  getSdfParam(sdf, "maxVoltage", max_voltage_, POSITIVE);
  getSdfParam(sdf, "sagVoltage", sag_voltage_, NON_NEGATIVE);
  getSdfParam(sdf, "maxCurrent", max_current_, POSITIVE);
  getSdfParam(sdf, "currentCapacity", capacity_, POSITIVE);
  getSdfParam(sdf, "internalRegistance", registance_, NON_NEGATIVE);
  getSdfParam(sdf, "voltageNoiseStddev", voltage_noise_stddev_, kDefaultVoltageNoiseStddev, NON_NEGATIVE);
  getSdfParam(sdf, "currentNoiseStddev", current_noise_stddev_, kDefaultCurrentNoiseStddev, NON_NEGATIVE);
  getSdfParam(sdf, "numRotors", num_rotors_, NON_NEGATIVE);
}

void GazeboBatteryPlugin::registerPubSub()
{
  battery_pub_ = createPublisher<tobas_msgs::msg::Battery>(path::join(ns(), tobas::kBatteryTopic));
  battery_gt_pub_ = createPublisher<tobas_msgs::msg::Battery>(path::join(ns(), kBatteryGtTopic));

  // モータ状態のコールバックとサブスクライバを設定
  for (size_t i = 0; i < num_rotors_; ++i)
  {
    const string suffix = "_" + to_string(i);
    const string topic = path::join(ns(), kRotorStateGtTopicPrefix + suffix);
    const auto cb = [this, i](const tobas_msgs::msg::RotorState::ConstSharedPtr& msg)
    {
      TOBAS_INFO("hoge");
      currents_[i] = msg->current;
    };
    const auto sub = node_->create_subscription<tobas_msgs::msg::RotorState>(topic, 1, cb);
    rotor_state_subs_.push_back(sub);
  }
}

void GazeboBatteryPlugin::PostUpdate(const sim::UpdateInfo& info, const sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime))
    return;

  // 電流を計算
  const auto current = tobas_std::fsum(currents_);
  if (current > max_current_)
    TOBAS_WARN_THROTTLE(kWarnPeriod, "The battery current is over limit: ", current, " > ", max_current_, " [A]");
  const auto current_obs = current + current_noise_(rnd_gen_);  // 観測ノイズを受けた観測電流

  // 電気容量の減少
  const auto dt = chrono::duration<double>(info.dt).count();
  q_ = max(q_ - current * dt, 0.);

  // 電圧を計算
  const auto voltage_in = currentVoltage();                              // 内部電圧
  const auto voltage_out = max(voltage_in - registance_ * current, 0.);  // 内部抵抗による電圧降下
  const auto voltage_obs = voltage_out + voltage_noise_(rnd_gen_);       // 観測ノイズを受けた観測電圧

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
  battery_gt->current = current;
  battery_gt_pub_->publish(move(battery_gt));
}

double GazeboBatteryPlugin::currentVoltage()
{
  // memo: 2-50
  const auto rate = q_ / capacity_;
  if (rate < 0.)
    return 0.;
  else if (rate < kSagCapRate)
    return sag_voltage_ * rate / kSagCapRate;
  else
    return (max_voltage_ - sag_voltage_) * (rate - kSagCapRate) / (1 - kSagCapRate) + sag_voltage_;
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
  sim::System,
  gazebo::GazeboBatteryPlugin::ISystemConfigure,
  gazebo::GazeboBatteryPlugin::ISystemPostUpdate)
