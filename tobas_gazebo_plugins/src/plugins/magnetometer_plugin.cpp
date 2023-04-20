#include <dh_std_tools/math.hpp>

#include "../../include/plugins/magnetometer_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"
#include "../../include/tobas_gazebo_plugins/conversions.hpp"

#define ZERO_3 (SdfVector3(0., 0., 0.))

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboMagnetometerPlugin::GazeboMagnetometerPlugin() : SensorPlugin(), rnd_gen_(rnd_dev_())
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
  noise_[0] = NormalDistribution(0, noise_normal_.X());
  noise_[1] = NormalDistribution(0, noise_normal_.Y());
  noise_[2] = NormalDistribution(0, noise_normal_.Z());

  // Create the uniform noise distribution for initial bias
  UniformDistribution initial_bias[3];
  initial_bias[0] =
    UniformDistribution(-noise_uniform_initial_bias_.X(), noise_uniform_initial_bias_.X());
  initial_bias[1] =
    UniformDistribution(-noise_uniform_initial_bias_.Y(), noise_uniform_initial_bias_.Y());
  initial_bias[2] =
    UniformDistribution(-noise_uniform_initial_bias_.Z(), noise_uniform_initial_bias_.Z());

  // Initialize the reference magnetic field vector in NWU world frame
  mag_NWU_ = Vector3d(
    ref_mag_north_ + initial_bias[0](rnd_gen_), -ref_mag_east_ + initial_bias[1](rnd_gen_),
    -ref_mag_down_ + initial_bias[2](rnd_gen_));

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
  if (!getSdfParam<string>(sdf, "robotNamespace", ns_))
  {
    gzthrow(kPluginName << ": Please specify a robotNamespace.");
  }

  if (!getSdfParam<string>(sdf, "linkName", link_name_))
  {
    gzthrow(kPluginName << ": Please specify a linkName.");
  }

  getSdfParam<string>(sdf, "magnetometerTopic", mag_topic_, kDefaultMagTopic);

  getSdfParam<double>(sdf, "refMagNorth", ref_mag_north_, kDefaultRefMagNorth);
  getSdfParam<double>(sdf, "refMagEast", ref_mag_east_, kDefaultRefMagEast);
  getSdfParam<double>(sdf, "refMagDown", ref_mag_down_, kDefaultRefMagDown);

  getSdfParam<SdfVector3>(sdf, "noiseNormal", noise_normal_, ZERO_3);
  getSdfParam<SdfVector3>(sdf, "noiseUniformInitialBias", noise_uniform_initial_bias_, ZERO_3);
  if (!allGreaterEqual(noise_normal_, 0.) || !allGreaterEqual(noise_uniform_initial_bias_, 0.))
  {
    gzthrow(kPluginName << ": Noise std. dev cannot be negative.");
  }
}

void GazeboMagnetometerPlugin::onUpdate()
{
  // Get the current pose and time from Gazebo
  Pose3d T_W_B = link_->WorldPose();
  common::Time cur_time = world_->SimTime();

  // Calculate the magnetic field noise
  Vector3d mag_noise(noise_[0](rnd_gen_), noise_[1](rnd_gen_), noise_[2](rnd_gen_));

  // Rotate the earth magnetic field into the inertial frame
  Vector3d field_B = T_W_B.Rot().RotateVectorReverse(mag_NWU_ + mag_noise);

  // Add noise to the true values
  addNoise(field_B);

  // Fill the magnetic field message
  timeGazeboToRos(cur_time, mag_msg_.header.stamp);
  vectorGazeboToRos(field_B, mag_msg_.magnetic_field);

  // Publish the message
  mag_pub_.publish(mag_msg_);
}

void GazeboMagnetometerPlugin::addNoise(Vector3d& mag)
{
  // TODO
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboMagnetometerPlugin);
}  // namespace gazebo
