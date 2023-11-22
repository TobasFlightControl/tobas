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

  registerPubSub();
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
}

void GazeboBatteryPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "maxVoltage", max_voltage_, POSITIVE);
  getSdfParam(sdf, "maxCurrent", max_current_, POSITIVE);
  getSdfParam(sdf, "numRotors", num_rotors_, NON_NEGATIVE);
}

void GazeboBatteryPlugin::registerPubSub()
{
  const string prefix = "/" + ns_ + "/";

  battery_pub_ = nh_.advertise<tobas_msgs::Battery>(prefix + tobas::kBatteryTopic, 1);

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
  // TODO: バッテリーの充放電モデル
  // TODO: 放電限界電圧になったらエラー

  // 電流
  const auto current_sum = dh_std::sum(currents_);
  if (current_sum > max_current_)
  {
    gzwarn << kPluginName << ": The battery current is over limit: " << current_sum << " > "
           << max_current_ << " [A]" << endl;
  }

  const auto battery_msg = boost::make_shared<tobas_msgs::Battery>();
  timeGazeboToRos(info.simTime, battery_msg->header.stamp);
  battery_msg->voltage = max_voltage_;
  battery_msg->current = current_sum;
  battery_pub_.publish(battery_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBatteryPlugin);
}  // namespace gazebo
