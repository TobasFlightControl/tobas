#include <tobas_std_tools/vector.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Battery.h>

#include "./battery_plugin.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;

namespace gazebo
{
GazeboBatteryPlugin::GazeboBatteryPlugin() : super(), rnd_gen_(rnd_dev_())
{
}

void GazeboBatteryPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  currents_.resize(num_rotors_, 0.);
  q_ = capacity_;
  voltage_noise_ = NormalDistribution(0., voltage_noise_stddev_);
  current_noise_ = NormalDistribution(0., current_noise_stddev_);

  registerPubSub();
  charge_srv_ = node_.advertiseService("/" + ns_ + "/" + kChargeBatterySrv, &self::chargeCb, this);

  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&self::onUpdate, this, _1));
}

void GazeboBatteryPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
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
  const string prefix = "/" + ns_ + "/";

  battery_pub_ = node_.advertise<tobas_msgs::Battery>(prefix + tobas::kBatteryTopic, 1);
  battery_gt_pub_ = node_.advertise<tobas_msgs::Battery>(prefix + kBatteryGtTopic, 1);

  // モータ状態のコールバックとサブスクライバを設定
  for (size_t i = 0; i < num_rotors_; ++i)
  {
    rotor_state_cbs_.push_back([this, i](const tobas_msgs::RotorStateConstPtr& msg) { currents_[i] = msg->current; });

    const string suffix = "_" + to_string(i);
    const string topic = prefix + kRotorStateGtTopicPrefix + suffix;
    rotor_state_subs_.push_back(node_.subscribe<tobas_msgs::RotorState>(topic, 1, rotor_state_cbs_[i]));
  }
}

void GazeboBatteryPlugin::onUpdate(const common::UpdateInfo& info)
{
  // 時刻を更新
  const auto ts = (info.simTime - t_last_).Double();  // サンプリング周期
  t_last_ = info.simTime;

  // 電流を計算
  const auto current = tobas_std::fsum(currents_);
  if (current > max_current_)
  {
    GZ_WARN_THROTTLE(
      kWarnPeriod,
      kPluginName << ": The battery current is over limit: " << current << " > " << max_current_ << " [A]" << endl);
  }
  const auto current_obs = current + current_noise_(rnd_gen_);  // 観測ノイズを受けた観測電流

  // 電気容量の減少
  q_ = max(q_ - current * ts, 0.);

  // 電圧を計算
  const auto voltage_in = currentVoltage();                              // 内部電圧
  const auto voltage_out = max(voltage_in - registance_ * current, 0.);  // 内部抵抗による電圧降下
  const auto voltage_obs = voltage_out + voltage_noise_(rnd_gen_);       // 観測ノイズを受けた観測電圧

  // 観測したバッテリーの状態を発行
  const auto battery = boost::make_shared<tobas_msgs::Battery>();
  timeGazeboToRos(info.simTime, battery->header.stamp);
  battery->voltage = voltage_obs;
  battery->current = current_obs;
  battery_pub_.publish(battery);

  // 真のバッテリーの状態を発行
  const auto battery_gt = boost::make_shared<tobas_msgs::Battery>();
  timeGazeboToRos(info.simTime, battery_gt->header.stamp);
  battery_gt->voltage = voltage_out;
  battery_gt->current = current;
  battery_gt_pub_.publish(battery_gt);
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

bool GazeboBatteryPlugin::chargeCb(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
  q_ = capacity_;
  gzmsg << kPluginName << ": Battery is charged." << endl;
  return true;
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBatteryPlugin);
}  // namespace gazebo
