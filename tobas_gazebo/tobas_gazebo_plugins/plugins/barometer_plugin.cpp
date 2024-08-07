#include <sensor_msgs/FluidPressure.h>

#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_constants/constants.hpp>

#include "./barometer_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboBarometerPlugin::GazeboBarometerPlugin() : super(), rnd_gen_(rnd_dev_())
{
}

void GazeboBarometerPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  // Get SDF parameters
  getSdfParams(sdf);

  // Get the world model
  world_ = physics::get_world(sensor->WorldName());

  // Get the pointer to the link
  link_ = dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name_));
  if (link_ == nullptr)
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  // Initialize the normal distribution for pressure
  pressure_noise_ = NormalDistribution(0., sqrt(pressure_var_));

  // Advertise
  pressure_pub_ = node_.advertise<PressureMsg>("/" + ns_ + "/" + tobas::kAirPressureTopic, 1);

  // Listen to the update event
  update_connection_ = sensor->ConnectUpdated(std::bind(&GazeboBarometerPlugin::onUpdate, this));
}

void GazeboBarometerPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "offset", offset_, zero3);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero, NON_NEGATIVE);
  getSdfParam(sdf, "pressureVariance", pressure_var_, kDefaultPressureVar, NON_NEGATIVE);
}

void GazeboBarometerPlugin::onUpdate()
{
  // Get the current geometric height of sensor
  const auto& T_W_B = link_->WorldPose();
  const auto& W_Pos_WB = T_W_B.Pos();
  const auto& W_Rot_B = T_W_B.Rot();
  const auto W_Pos_WS = W_Pos_WB + W_Rot_B * offset_;
  const auto altitude = alt_0_ + W_Pos_WS.Z();

  // Compute the air pressure at the current altitude
  auto pressure = tobas_std::altitudeToPressure(altitude);

  // Add noise to pressure measurement
  pressure += pressure_noise_(rnd_gen_);

  // Create a pressure message
  const auto pressure_msg = make_unique<sensor_msgs::msg::FluidPressure>();
  timeGazeboToRos(world_->SimTime(), pressure_msg->header.stamp);
  pressure_msg->header.frame_id = link_name_;
  pressure_msg->fluid_pressure = pressure;
  pressure_msg->variance = pressure_var_;

  // Publish the pressure message
  pressure_pub_.publish(pressure_msg);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboBarometerPlugin);
}  // namespace gazebo
