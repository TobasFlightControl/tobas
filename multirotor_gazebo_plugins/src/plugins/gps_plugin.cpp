#include <dh_std_tools/math.hpp>

#include "../../include/plugins/gps_plugin.hpp"
#include "../../include/multirotor_gazebo_plugins/utils.hpp"
#include "../../include/multirotor_gazebo_plugins/conversions.hpp"

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboGpsPlugin::GazeboGpsPlugin() : SensorPlugin(), rnd_gen_(rnd_dev_())
{
}

void GazeboGpsPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
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

  // Initialize the normal distributions
  pos_noise_[0] = NormalDistribution(0., hor_pos_std_dev_);
  pos_noise_[1] = NormalDistribution(0., hor_pos_std_dev_);
  pos_noise_[2] = NormalDistribution(0., ver_pos_std_dev_);
  vel_noise_[0] = NormalDistribution(0., hor_vel_std_dev_);
  vel_noise_[1] = NormalDistribution(0., hor_vel_std_dev_);
  vel_noise_[2] = NormalDistribution(0., ver_vel_std_dev_);

  // Fill the static parts of the GPS message
  pos_msg_.header.frame_id = link_name_;
  pos_msg_.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  pos_msg_.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
  pos_msg_.position_covariance_type = PosMsg::COVARIANCE_TYPE_KNOWN;

  pos_msg_.position_covariance[0] = sqr(hor_pos_std_dev_);
  pos_msg_.position_covariance[1] = 0.;
  pos_msg_.position_covariance[2] = 0.;
  pos_msg_.position_covariance[3] = 0.;
  pos_msg_.position_covariance[4] = sqr(hor_pos_std_dev_);
  pos_msg_.position_covariance[5] = 0.;
  pos_msg_.position_covariance[6] = 0.;
  pos_msg_.position_covariance[7] = 0.;
  pos_msg_.position_covariance[8] = sqr(ver_pos_std_dev_);

  // Fill the static parts of the ground speed message
  vel_msg_.header.frame_id = link_name_;

  vel_msg_.vel.covariance[0] = sqr(hor_vel_std_dev_);
  vel_msg_.vel.covariance[1] = 0.;
  vel_msg_.vel.covariance[2] = 0.;
  vel_msg_.vel.covariance[3] = 0.;
  vel_msg_.vel.covariance[4] = sqr(hor_vel_std_dev_);
  vel_msg_.vel.covariance[5] = 0.;
  vel_msg_.vel.covariance[6] = 0.;
  vel_msg_.vel.covariance[7] = 0.;
  vel_msg_.vel.covariance[8] = sqr(ver_vel_std_dev_);

  // Advertise publishers
  pos_pub_ = nh_.advertise<PosMsg>("/" + ns_ + "/" + gps_topic_, 1);
  vel_pub_ = nh_.advertise<VelMsg>("/" + ns_ + "/" + vel_topic_, 1);

  // Connect to the sensor update event
  update_connection_ = sensor->ConnectUpdated(boost::bind(&GazeboGpsPlugin::onUpdate, this));
}

void GazeboGpsPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  if (!getSdfParam<string>(sdf, "robotNamespace", ns_))
  {
    gzthrow(kPluginName << ": Please specify a robotNamespace.");
  }

  if (!getSdfParam<string>(sdf, "linkName", link_name_))
  {
    gzthrow(kPluginName << ": Please specify a linkName.");
  }

  getSdfParam<string>(sdf, "gpsTopic", gps_topic_, kDefaultGpsTopic);
  getSdfParam<string>(sdf, "groundSpeedTopic", vel_topic_, kDefaultGroundSpeedTopic);

  getSdfParam<double>(sdf, "horPosStdDev", hor_pos_std_dev_, kDefaultHorPosStdDev);
  getSdfParam<double>(sdf, "verPosStdDev", ver_pos_std_dev_, kDefaultVerPosStdDev);
  getSdfParam<double>(sdf, "horVelStdDev", hor_vel_std_dev_, kDefaultHorVelStdDev);
  getSdfParam<double>(sdf, "verVelStdDev", ver_vel_std_dev_, kDefaultVerVelStdDev);
  if (
    hor_pos_std_dev_ < 0. || ver_pos_std_dev_ < 0. || hor_vel_std_dev_ < 0.
    || ver_vel_std_dev_ < 0.)
  {
    gzthrow(kPluginName << ": Noise std. dev cannot be negative.");
  }

  getSdfParam<double>(sdf, "latitudeZero", lat_0_, kDefaultLatitudeZero);
  getSdfParam<double>(sdf, "longitudeZero", lon_0_, kDefaultLatitudeZero);
}

void GazeboGpsPlugin::onUpdate()
{
  updatePosition();
  updateVelocity();

  pos_pub_.publish(pos_msg_);
  vel_pub_.publish(vel_msg_);
}

void GazeboGpsPlugin::updatePosition()
{
  // Get the time of the last measurement
  common::Time cur_time = world_->SimTime();

  // Get the position in the world frame
  Vector3 pos = link_->WorldPose().Pos();

  // Apply noise to the position
  pos += Vector3d(pos_noise_[0](rnd_gen_), pos_noise_[1](rnd_gen_), pos_noise_[2](rnd_gen_));

  // Fill the GPS message
  pos_msg_.header.stamp.sec = cur_time.sec;
  pos_msg_.header.stamp.nsec = cur_time.nsec;
  dh_std::cartToGpsRelative(
    pos.X(), pos.Y(), lat_0_, lon_0_, pos_msg_.latitude, pos_msg_.longitude);
  pos_msg_.altitude = pos.Z();
}

void GazeboGpsPlugin::updateVelocity()
{
  // Get the time of the last measurement
  common::Time cur_time = world_->SimTime();

  // Get the linear velocity in the world frame
  Vector3d vel = link_->WorldLinearVel();

  // Apply noise to ground speed
  vel += Vector3d(vel_noise_[0](rnd_gen_), vel_noise_[1](rnd_gen_), vel_noise_[2](rnd_gen_));

  // Fill the ground speed message.
  timeGazeboToRos(cur_time, vel_msg_.header.stamp);
  linvelGazeboToRos(vel, vel_msg_.vel.vel);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboGpsPlugin);
}  // namespace gazebo
