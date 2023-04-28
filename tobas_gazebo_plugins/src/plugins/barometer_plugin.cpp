#include "../../include/plugins/barometer_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"
#include "../../include/tobas_gazebo_plugins/conversions.hpp"
#include "../../include/tobas_gazebo_plugins/constants.hpp"

using namespace std;

namespace gazebo
{
GazeboBarometerPlugin::GazeboBarometerPlugin() : super(), rnd_gen_(rnd_dev_())
{
}

void GazeboBarometerPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  // Get SDF parameters
  getSdfParams(sdf);

  // Get the world model
  world_ = physics::get_world(sensor->WorldName());

  // Get the pointer to the link
  link_ = dynamic_pointer_cast<physics::Link>(world_->EntityByName(link_name_));
  if (link_ == NULL)
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  // Initialize the normal distribution for pressure
  pressure_noise_ = NormalDistribution(0., sqrt(pressure_var_));

  // Fill the static parts of the barometer message
  pressure_msg_.header.frame_id = link_name_;
  pressure_msg_.variance = pressure_var_;

  // Advertise
  pressure_pub_ = nh_.advertise<PressureMsg>("/" + ns_ + "/" + pressure_topic_, 1);

  // Listen to the update event
  update_connection_ = sensor->ConnectUpdated(boost::bind(&GazeboBarometerPlugin::onUpdate, this));
}

void GazeboBarometerPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  if (!getSdfParam<string>(sdf, "robotNamespace", ns_))
  {
    gzthrow(kPluginName << ": Please specify a robotNamespace.");
  }

  if (!getSdfParam<string>(sdf, "linkName", link_name_))
  {
    gzthrow(kPluginName << ": Please specify a linkName.");
  }

  getSdfParam<string>(sdf, "pressureTopic", pressure_topic_, kDefaultPressurePubTopic);
  getSdfParam<double>(sdf, "referenceAltitude", ref_alt_, kDefaultReferenceAltitude);

  getSdfParam<double>(sdf, "pressureVariance", pressure_var_, kDefaultPressureVar);
  if (pressure_var_ < 0.)
  {
    gzthrow(kPluginName << ": Noise variance cannot be negative.");
  }
}

void GazeboBarometerPlugin::onUpdate()
{
  common::Time cur_time = world_->SimTime();

  // Get the current geometric height
  double height_geometric_m = ref_alt_ + link_->WorldPose().Pos().Z();

  // Compute the geopotential height
  double height_geopotential_m =
    kEarthRadiusMeters * height_geometric_m / (kEarthRadiusMeters + height_geometric_m);

  // Compute the temperature at the current altitude
  double temperature_at_altitude_kelvin =
    kSeaLevelTempKelvin - kTempLapseKelvinPerMeter * height_geopotential_m;

  // Compute the current air pressure
  double pressure_at_altitude_pascal =
    kPressureOneAtmospherePascals
    * exp(kAirConstantDimensionless * log(kSeaLevelTempKelvin / temperature_at_altitude_kelvin));

  // Add noise to pressure measurement
  pressure_at_altitude_pascal += pressure_noise_(rnd_gen_);

  // Fill the pressure message
  timeGazeboToRos(cur_time, pressure_msg_.header.stamp);
  pressure_msg_.fluid_pressure = pressure_at_altitude_pascal;

  // Publish the pressure message
  pressure_pub_.publish(pressure_msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboBarometerPlugin);
}  // namespace gazebo
