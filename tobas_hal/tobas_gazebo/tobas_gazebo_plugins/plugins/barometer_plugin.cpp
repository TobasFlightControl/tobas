#include <tobas_std_tools/standard_atmosphere.hpp>

#include <tobas_tools/constants.hpp>

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
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  // Initialize the normal distribution for pressure
  pressure_noise_ = NormalDistribution(0., sqrt(pressure_var_));

  // Fill the static parts of the barometer message
  pressure_msg_.header.frame_id = link_name_;
  pressure_msg_.variance = pressure_var_;

  // Advertise
  pressure_pub_ = nh_.advertise<PressureMsg>("/" + ns_ + "/" + tobas::kAirPressureTopic, 1);

  // Listen to the update event
  update_connection_ = sensor->ConnectUpdated(boost::bind(&GazeboBarometerPlugin::onUpdate, this));
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
  const Pose3d& T_W_B = link_->WorldPose();
  const Vector3d& W_Pos_WB = T_W_B.Pos();
  const Quaterniond& W_Rot_B = T_W_B.Rot();
  const Vector3d W_Pos_WS = W_Pos_WB + W_Rot_B * offset_;
  const double altitude = alt_0_ + W_Pos_WS.Z();

  // Compute the air pressure at the current altitude
  double pressure = tobas_std::altitudeToPressure(altitude);

  // Add noise to pressure measurement
  pressure += pressure_noise_(rnd_gen_);

  // Fill the pressure message
  timeGazeboToRos(world_->SimTime(), pressure_msg_.header.stamp);
  pressure_msg_.fluid_pressure = pressure;

  // Publish the pressure message
  pressure_pub_.publish(pressure_msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboBarometerPlugin);
}  // namespace gazebo
