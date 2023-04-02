#include "../../include/plugins/pressure_plugin.hpp"
#include "../../include/multirotor_gazebo_plugins/utils.hpp"
#include "../../include/multirotor_gazebo_plugins/conversions.hpp"

using namespace std;

namespace gazebo
{
GazeboPressurePlugin::GazeboPressurePlugin() : ModelPlugin(), rnd_gen_(rnd_dev_())
{
}

void GazeboPressurePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get SDF parameters
  getSdfParams(sdf);

  // Store the pointer to the model and the world
  model_ = model;
  world_ = model_->GetWorld();

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
  {
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");
  }

  // Initialize the normal distribution for pressure
  pressure_noise_ = NormalDistribution(0., sqrt(pressure_var_));

  // Fill the static parts of the barometer message
  pressure_msg_.header.frame_id = link_name_;
  pressure_msg_.variance = pressure_var_;

  // Advertise publisher
  pressure_pub_ = nh_.advertise<PressureMsg>("/" + ns_ + "/" + pressure_topic_, 1);

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboPressurePlugin::onUpdate, this, _1));
}

void GazeboPressurePlugin::getSdfParams(sdf::ElementPtr sdf)
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
  getSdfParam<double>(sdf, "referenceAltitude", ref_alt_, kDefaultRefAlt);

  getSdfParam<double>(sdf, "pressureVariance", pressure_var_, kDefaultPressureVar);
  if (pressure_var_ < 0.)
  {
    gzthrow(kPluginName << ": Noise variance cannot be negative.");
  }
}

void GazeboPressurePlugin::onUpdate(const common::UpdateInfo&)
{
  common::Time cur_time = world_->SimTime();

  // Get the current geometric height
  double height_geometric_m = ref_alt_ + model_->WorldPose().Pos().Z();

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

GZ_REGISTER_MODEL_PLUGIN(GazeboPressurePlugin);
}  // namespace gazebo
