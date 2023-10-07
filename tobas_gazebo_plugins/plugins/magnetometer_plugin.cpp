#include <dh_std_tools/math.hpp>
#include <dh_std_tools/geometry.hpp>

#include <tobas_tools/constants.hpp>
#include <tobas_tools/utils.hpp>

#include "./magnetometer_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboMagnetometerPlugin::GazeboMagnetometerPlugin() : super()
{
}

void GazeboMagnetometerPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

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

  // Create the initial bias
  UniformDistribution3d init_bias_dist(
    rnd_dev_, -noise_uniform_initial_bias_, noise_uniform_initial_bias_);
  init_bias_ = init_bias_dist.get();

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
  mag_pub_ = nh_.advertise<MagMsg>("/" + ns_ + "/" + tobas::kMagTopic, 1);

  // Listen to the update event
  update_connection_ =
    sensor->ConnectUpdated(boost::bind(&GazeboMagnetometerPlugin::onUpdate, this));
}

void GazeboMagnetometerPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "offset", offset_, zero3);

  getSdfParam(sdf, "latitudeZero", lat_0_, kDefaultLatitudeZero);
  getSdfParam(sdf, "longitudeZero", lon_0_, kDefaultLongitudeZero);
  getSdfParam(sdf, "altitudeZero", alt_0_, kDefaultAltitudeZero);

  getSdfParam(sdf, "noiseNormal", noise_normal_, zero3);
  getSdfParam(sdf, "noiseUniformInitialBias", noise_uniform_initial_bias_, zero3);
  if (!allGreaterEqual(noise_normal_, 0.) || !allGreaterEqual(noise_uniform_initial_bias_, 0.))
  {
    gzthrow(kPluginName << ": Noise std. dev cannot be negative.");
  }
}

void GazeboMagnetometerPlugin::onUpdate()
{
  // Get the sensor pose
  const Pose3d& T_W_B = link_->WorldPose();
  const Vector3d& W_Pos_WB = T_W_B.Pos();
  const Quaterniond& W_Rot_B = T_W_B.Rot();
  const Vector3d W_Pos_WS = W_Pos_WB + W_Rot_B * offset_;

  // デカルト座標から経緯度と高度を計算
  dh_std::cartToGpsRelative(W_Pos_WS.X(), W_Pos_WS.Y(), lat_0_, lon_0_, lat_, lon_);
  const auto alt = alt_0_ + W_Pos_WS.Z();

  // 経緯度と高度から地磁気の参照値を計算
  const auto mag = tobas::geomag(lat_, lon_, alt);

  // 機体座標系から見た地磁気を計算
  Vector3d mag_W(mag.north, -mag.east, -mag.down);  // [nT]
  auto field_B = T_W_B.Rot().RotateVectorReverse(mag_W + init_bias_);

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
