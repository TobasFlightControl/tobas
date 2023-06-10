#include <dh_std_tools/math.hpp>

#include "../../include/plugins/magnetometer_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboMagnetometerPlugin::GazeboMagnetometerPlugin() : super()
{
}

void GazeboMagnetometerPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
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

  // Create the normal noise distributions
  noise_.reset(new NormalDistribution3d(rnd_dev_, zero3, noise_normal_));

  // Create the uniform noise distribution for initial bias
  UniformDistribution3d initial_bias(
    rnd_dev_, -noise_uniform_initial_bias_, noise_uniform_initial_bias_);

  // Initialize the reference magnetic field vector in NWU world frame
  mag_NWU_ = Vector3d(ref_mag_north_, -ref_mag_east_, -ref_mag_down_);
  mag_NWU_ += initial_bias.get();

  // Fill the static parts of the magnetometer message
  mag_msg_.header.frame_id = link_name_;

  mag_msg_.magnetic_field_covariance[0] = dh_std::sqr(noise_normal_.X());
  mag_msg_.magnetic_field_covariance[1] = 0.;
  mag_msg_.magnetic_field_covariance[2] = 0.;
  mag_msg_.magnetic_field_covariance[3] = 0.;
  mag_msg_.magnetic_field_covariance[4] = dh_std::sqr(noise_normal_.Y());
  mag_msg_.magnetic_field_covariance[5] = 0.;
  mag_msg_.magnetic_field_covariance[6] = 0.;
  mag_msg_.magnetic_field_covariance[7] = 0.;
  mag_msg_.magnetic_field_covariance[8] = dh_std::sqr(noise_normal_.Z());

  // Advertise publisher
  mag_pub_ = nh_.advertise<MagMsg>("/" + ns_ + "/" + mag_topic_, 1);

  // Listen to the update event
  update_connection_ =
    sensor->ConnectUpdated(boost::bind(&GazeboMagnetometerPlugin::onUpdate, this));
}

void GazeboMagnetometerPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);

  getSdfParam(sdf, "magnetometerTopic", mag_topic_, kDefaultMagTopic);

  getSdfParam(sdf, "refMagNorth", ref_mag_north_, kDefaultRefMagNorth);
  getSdfParam(sdf, "refMagEast", ref_mag_east_, kDefaultRefMagEast);
  getSdfParam(sdf, "refMagDown", ref_mag_down_, kDefaultRefMagDown);

  getSdfParam(sdf, "noiseNormal", noise_normal_, zero3);
  getSdfParam(sdf, "noiseUniformInitialBias", noise_uniform_initial_bias_, zero3);
  if (!allGreaterEqual(noise_normal_, 0.) || !allGreaterEqual(noise_uniform_initial_bias_, 0.))
  {
    gzthrow(kPluginName << ": Noise std. dev cannot be negative.");
  }
}

void GazeboMagnetometerPlugin::onUpdate()
{
  // Get the earth magnetic field wrt. the body frame
  const auto T_W_B = link_->WorldPose();
  auto field_B = T_W_B.Rot().RotateVectorReverse(mag_NWU_);

  // Add noise
  field_B += noise_->get();

  // Fill the magnetic field message
  timeGazeboToRos(world_->SimTime(), mag_msg_.header.stamp);
  vectorGazeboToRos(field_B, mag_msg_.magnetic_field);

  // Publish the message
  mag_pub_.publish(mag_msg_);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboMagnetometerPlugin);
}  // namespace gazebo
