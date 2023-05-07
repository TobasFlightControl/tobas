#include <dh_std_tools/math.hpp>

#include "../../include/plugins/imu_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"
#include "../../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

#define ZERO_3 (Vector3d(0., 0., 0.))

using namespace std;
using namespace ignition::math;

namespace gazebo
{
GazeboImuPlugin::GazeboImuPlugin() : super(), rnd_gen_(rnd_dev_()), velocity_prev_W_(0., 0., 0.)
{
}

void GazeboImuPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
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

  last_time_ = world_->SimTime();
  gravity_W_ = world_->Gravity();
  gyro_bias_ = ZERO_3;
  acc_bias_ = ZERO_3;

  noise_ = NormalDistribution(0., 1.);
  for (int i = 0; i < 3; ++i)
  {
    gyro_turn_on_bias_[i] = gyro_turn_on_bias_sigma_ * noise_(rnd_gen_);
    acc_turn_on_bias_[i] = acc_turn_on_bias_sigma_ * noise_(rnd_gen_);
  }

  // Fill the static parts of the imu message
  imu_msg_.header.frame_id = link_name_;

  imu_msg_.linear_acceleration_covariance.fill(0.);
  imu_msg_.angular_velocity_covariance.fill(0.);
  imu_msg_.orientation_covariance.fill(-1.);

  imu_msg_.orientation.x = -1.;
  imu_msg_.orientation.y = -1.;
  imu_msg_.orientation.z = -1.;
  imu_msg_.orientation.w = -1.;

  // Advertise
  imu_pub_ = nh_.advertise<ImuMsg>("/" + ns_ + "/" + imu_topic_, 1);

  // Listen to the update event
  update_connection_ = sensor->ConnectUpdated(boost::bind(&GazeboImuPlugin::onUpdate, this));
}

void GazeboImuPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);

  // TODO: 範囲チェック
  getSdfParam(sdf, "imuTopic", imu_topic_, kDefaultImuTopic);
  getSdfParam(sdf, "gyroscopeNoiseDensity", gyro_noise_density_, kDefaultGyroNoiseDensity);
  getSdfParam(sdf, "gyroscopeRandomWalk", gyro_random_walk_, kDefaultGyroRandomWalk);
  getSdfParam(sdf, "gyroscopeBiasCorrelationTime", gyro_bias_corr_time_, kDefaultGyroBiasCorrTime);
  getSdfParam(
    sdf, "gyroscopeTurnOnBiasSigma", gyro_turn_on_bias_sigma_, kDefaultGyroTurnOnBiasSigma);
  getSdfParam(sdf, "accelerometerNoiseDensity", acc_noise_density_, kDefaultAccNoiseDensity);
  getSdfParam(sdf, "accelerometerRandomWalk", acc_random_walk_, kDefaultAccRandomWalk);
  getSdfParam(
    sdf, "accelerometerBiasCorrelationTime", acc_bias_corr_time_, kDefaultAccBiasCorrTime);
  getSdfParam(
    sdf, "accelerometerTurnOnBiasSigma", acc_turn_on_bias_sigma_, kDefaultAccTurnOnBiasSigma);
}

void GazeboImuPlugin::onUpdate()
{
  common::Time cur_time = world_->SimTime();
  double dt = (cur_time - last_time_).Double();
  last_time_ = cur_time;

  // Get linear acceleration and angular velocity from simulation
  Pose3d T_W_B = link_->WorldPose();
  Quaterniond R_W_B = T_W_B.Rot();
  Vector3d acc_B = link_->RelativeLinearAccel() - R_W_B.RotateVectorReverse(gravity_W_);
  Vector3d gyro_B = link_->RelativeAngularVel();

  // Add noise to the true values
  addNoise(acc_B, gyro_B, dt);

  // Fill IMU message.
  timeGazeboToRos(cur_time, imu_msg_.header.stamp);
  vectorGazeboToRos(acc_B, imu_msg_.linear_acceleration);
  vectorGazeboToRos(gyro_B, imu_msg_.angular_velocity);

  double acc_var = dh_std::sqr(acc_noise_density_) / dt;
  fillMatrix3Diag(imu_msg_.linear_acceleration_covariance, acc_var);

  double gyro_var = dh_std::sqr(gyro_noise_density_) / dt;
  fillMatrix3Diag(imu_msg_.angular_velocity_covariance, gyro_var);

  // Publish IMU message
  imu_pub_.publish(imu_msg_);
}

void GazeboImuPlugin::addNoise(Vector3d& lin_acc, Vector3d& ang_vel, double dt)
{
  // Gyrosocpe
  double tau_g = gyro_bias_corr_time_;
  // Discrete-time std. dev equivalent to an "integrating" sampler with integration time dt.
  double sigma_g_d = 1 / sqrt(dt) * gyro_noise_density_;
  double sigma_b_g = gyro_random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_g_d = sqrt(-sigma_b_g * sigma_b_g * tau_g / 2. * (exp(-2. * dt / tau_g) - 1.));
  // Compute state-transition.
  double phi_g_d = exp(-1. / tau_g * dt);
  // Simulate gyroscope noise processes and add them to the true angular rate.
  for (int i = 0; i < 3; ++i)
  {
    gyro_bias_[i] = phi_g_d * gyro_bias_[i] + sigma_b_g_d * noise_(rnd_gen_);
    ang_vel[i] = ang_vel[i] + gyro_bias_[i] + sigma_g_d * noise_(rnd_gen_) + gyro_turn_on_bias_[i];
  }

  // Accelerometer
  double tau_a = acc_bias_corr_time_;
  // Discrete-time std. dev equivalent to an "integrating" sampler with integration time dt.
  double sigma_a_d = 1 / sqrt(dt) * acc_noise_density_;
  double sigma_b_a = acc_random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114].
  double sigma_b_a_d = sqrt(-sigma_b_a * sigma_b_a * tau_a / 2. * (exp(-2. * dt / tau_a) - 1.));
  // Compute state-transition.
  double phi_a_d = exp(-1. / tau_a * dt);
  // Simulate accelerometer noise processes and add them to the true linear acceleration.
  for (int i = 0; i < 3; ++i)
  {
    acc_bias_[i] = phi_a_d * acc_bias_[i] + sigma_b_a_d * noise_(rnd_gen_);
    lin_acc[i] = lin_acc[i] + acc_bias_[i] + sigma_a_d * noise_(rnd_gen_) + acc_turn_on_bias_[i];
  }
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboImuPlugin);
}  // namespace gazebo
