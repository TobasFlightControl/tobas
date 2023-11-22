#include <dh_std_tools/vector.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Battery.h>

#include "./battery_plugin.hpp"
#include "../include/tobas_gazebo_plugins/common.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;

namespace gazebo
{
GazeboBatteryPlugin::GazeboBatteryPlugin() : super()
{
}

void GazeboBatteryPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  getSdfParams(sdf);

  currents_.resize(num_rotors_, 0.);
  q_ = capacity_;

  registerPubSub();
  charge_srv_ = nh_.advertiseService("/" + ns_ + "/charge_battery", &self::chargeCb, this);

  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
}

void GazeboBatteryPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "maxVoltage", max_voltage_, POSITIVE);
  getSdfParam(sdf, "sagVoltage", sag_voltage_, NON_NEGATIVE);
  getSdfParam(sdf, "maxCurrent", max_current_, POSITIVE);
  getSdfParam(sdf, "currentCapacity", capacity_, POSITIVE);
  getSdfParam(sdf, "numRotors", num_rotors_, NON_NEGATIVE);
}

void GazeboBatteryPlugin::registerPubSub()
{
  const string prefix = "/" + ns_ + "/";

  battery_pub_ = nh_.advertise<tobas_msgs::Battery>(prefix + tobas::kBatteryTopic, 1);

  // モータ状態のコールバックとサブスクライバを設定
  for (size_t i = 0; i < num_rotors_; ++i)
  {
    rotor_state_cbs_.push_back(
      [this, i](const tobas_msgs::RotorStateConstPtr& msg) { currents_[i] = msg->current; });

    const string suffix = "_" + to_string(i);
    const string topic = prefix + kRotorStateGtTopicPrefix + suffix;
    rotor_state_subs_.push_back(
      nh_.subscribe<tobas_msgs::RotorState>(topic, 1, rotor_state_cbs_[i]));
  }
}

void GazeboBatteryPlugin::onUpdate(const common::UpdateInfo& info)
{
  // 時刻を更新
  const auto dt = (info.simTime - t_last_).Double();
  t_last_ = info.simTime;

  // 電流を計算
  const auto current = dh_std::sum(currents_);
  if (current > max_current_)
  {
    gzwarn << kPluginName << ": The battery current is over limit: " << current << " > "
           << max_current_ << " [A]" << endl;
  }

  // 電気容量の減少
  q_ = max(q_ - current * dt, 0.);

  // 電圧を計算
  const auto voltage = currentVoltage();

  // バッテリーの状態を発行
  const auto battery_msg = boost::make_shared<tobas_msgs::Battery>();
  timeGazeboToRos(info.simTime, battery_msg->header.stamp);
  battery_msg->voltage = voltage;
  battery_msg->current = current;
  battery_pub_.publish(battery_msg);
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
