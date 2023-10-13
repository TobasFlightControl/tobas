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

  noise_ = NormalDistribution(0, noise_stddev_);

  registerPubSub();
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&self::onUpdate, this, _1));
}

void GazeboBatteryPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "nominalVoltage", nominal_voltage_, POSITIVE);
  getSdfParam(sdf, "voltageNoiseStddev", noise_stddev_, kDefaultVoltageNoiseStddev, POSITIVE);
}

void GazeboBatteryPlugin::registerPubSub()
{
  battery_pub_ = nh_.advertise<tobas_msgs::Battery>("/" + ns_ + "/" + tobas::kBatteryTopic, 1);
  battery_gt_pub_ = nh_.advertise<tobas_msgs::Battery>("/" + ns_ + "/" + kBatteryGtTopic, 1);
}

void GazeboBatteryPlugin::onUpdate(const common::UpdateInfo& info)
{
  // TODO: バッテリーの充放電モデル
  // TODO: 放電限界電圧になったらエラー

  auto battery_msg = boost::make_shared<tobas_msgs::Battery>();
  auto battery_gt_msg = boost::make_shared<tobas_msgs::Battery>();

  timeGazeboToRos(info.simTime, battery_msg->header.stamp);
  timeGazeboToRos(info.simTime, battery_gt_msg->header.stamp);

  battery_msg->voltage = nominal_voltage_ + noise_(rnd_gen_);
  battery_gt_msg->voltage = nominal_voltage_;

  battery_pub_.publish(battery_msg);
  battery_gt_pub_.publish(battery_gt_msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBatteryPlugin);
}  // namespace gazebo
