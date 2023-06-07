#include "../../include/plugins/battery_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

namespace gazebo
{
GazeboBatteryPlugin::GazeboBatteryPlugin() : super()
{
}

void GazeboBatteryPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  getSdfParams(sdf);
  registerPubSub();
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboBatteryPlugin::onUpdate, this, _1));
}

void GazeboBatteryPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "batteryPubTopic", battery_pub_topic_, kDefaultBatteryPubTopic);
  getSdfParam(sdf, "nominalVoltage", nominal_voltage_, POSITIVE);
}

void GazeboBatteryPlugin::registerPubSub()
{
  battery_pub_ = nh_.advertise<tobas_msgs::Battery>("/" + ns_ + "/" + battery_pub_topic_, 1);
}

void GazeboBatteryPlugin::onUpdate(const common::UpdateInfo& info)
{
  // TODO: バッテリーの充放電モデル
  // TODO: 放電限界電圧になったらエラー
  timeGazeboToRos(info.simTime, battery_msg_.header.stamp);
  battery_msg_.voltage = nominal_voltage_;

  battery_pub_.publish(battery_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboBatteryPlugin);
}  // namespace gazebo
