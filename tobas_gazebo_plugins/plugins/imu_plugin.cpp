#include <dh_std_tools/math.hpp>
#include <dh_std_tools/boost.hpp>

#include <tobas_gazebo_plugins/ImuDebug.h>

#include "./imu_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace ignition::math;
using namespace dh_std;

namespace gazebo
{
GazeboImuPlugin::GazeboImuPlugin() : super(), rnd_gen_(rnd_dev_())
{
}

void GazeboImuPlugin::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf)
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

  noise_ = NormalDistribution(0, 1);
  for (uint32_t i = 0; i < 3; ++i)
  {
    acc_turn_on_bias_[i] = acc_turn_on_bias_sigma_ * noise_(rnd_gen_);
    gyro_turn_on_bias_[i] = gyro_turn_on_bias_sigma_ * noise_(rnd_gen_);
  }

  // Initialize LPFs
  const auto tau_acc_lpf = dh_std::timeConstFromCutoffFreq(acc_lpf_cutoff_freq_);
  const auto tau_gyro_lpf = dh_std::timeConstFromCutoffFreq(gyro_lpf_cutoff_freq_);
  acc_lpf_.initialize(tau_acc_lpf, zero3);
  gyro_lpf_.initialize(tau_gyro_lpf, zero3);

  // Fill the static parts of the imu message
  imu_msg_.header.frame_id = link_name_;

  imu_msg_.linear_acceleration_covariance.fill(0);
  imu_msg_.angular_velocity_covariance.fill(0);
  imu_msg_.orientation_covariance.fill(nan(tobas::kUnknown));

  imu_msg_.orientation.x = nan(tobas::kUnknown);
  imu_msg_.orientation.y = nan(tobas::kUnknown);
  imu_msg_.orientation.z = nan(tobas::kUnknown);
  imu_msg_.orientation.w = nan(tobas::kUnknown);

  // Advertise
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/" + ns_ + "/" + tobas::kImuTopic, 1);
  debug_pub_ = nh_.advertise<tobas_gazebo_plugins::ImuDebug>("/" + ns_ + "/" + kDebugPubTopic, 1);

  // Listen to the update event
  update_connection_ = sensor->ConnectUpdated(boost::bind(&self::onUpdate, this));
}

void GazeboImuPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam(sdf, "robotNamespace", ns_);
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "offset", offset_, zero3);

  getSdfParam(
    sdf, "accelNoiseDensityOnSignal", acc_noise_density_sig_, kDefaultAccNoiseDensity, POSITIVE);
  getSdfParam(
    sdf, "accelNoiseDensityObserved", acc_noise_density_obs_, kDefaultAccNoiseDensity, POSITIVE);
  getSdfParam(sdf, "accelRandomWalk", acc_random_walk_, kDefaultAccRandomWalk, POSITIVE);
  getSdfParam(
    sdf, "accelBiasCorrelationTime", acc_bias_corr_time_, kDefaultAccBiasCorrTime, POSITIVE);
  getSdfParam(
    sdf, "accelTurnOnBiasSigma", acc_turn_on_bias_sigma_, kDefaultAccTurnOnBiasSigma, POSITIVE);
  getSdfParam(sdf, "accelLpfCutoffFreq", acc_lpf_cutoff_freq_, kDefaultAccLpfCutoffFreq, POSITIVE);

  getSdfParam(
    sdf, "gyroNoiseDensityOnSignal", gyro_noise_density_sig_, kDefaultGyroNoiseDensity, POSITIVE);
  getSdfParam(
    sdf, "gyroNoiseDensityObserved", gyro_noise_density_obs_, kDefaultGyroNoiseDensity, POSITIVE);
  getSdfParam(sdf, "gyroRandomWalk", gyro_random_walk_, kDefaultGyroRandomWalk, POSITIVE);
  getSdfParam(
    sdf, "gyroBiasCorrelationTime", gyro_bias_corr_time_, kDefaultGyroBiasCorrTime, POSITIVE);
  getSdfParam(
    sdf, "gyroTurnOnBiasSigma", gyro_turn_on_bias_sigma_, kDefaultGyroTurnOnBiasSigma, POSITIVE);
  getSdfParam(sdf, "gyroLpfCutoffFreq", gyro_lpf_cutoff_freq_, kDefaultGyroLpfCutoffFreq, POSITIVE);
}

void GazeboImuPlugin::onUpdate()
{
  const common::Time cur_time = world_->SimTime();
  const double dt = (cur_time - last_time_).Double();
  last_time_ = cur_time;

  // ベースフレームの状態を取得
  const Pose3d& T_W_B = link_->WorldPose();
  const Quaterniond& R_W_B = T_W_B.Rot();
  const Vector3d acc_B = link_->RelativeLinearAccel();
  const Vector3d omega_B = link_->RelativeAngularVel();
  const Vector3d domega_B = link_->RelativeAngularAccel();

  // オフセットによる補正を考慮して加速度センサの読みを計算 (memo: 2-26)
  const Vector3d grav_B = R_W_B.RotateVectorReverse(world_->Gravity());
  const Vector3d acc_corr = omega_B.Cross(omega_B.Cross(offset_)) + domega_B.Cross(offset_);
  Vector3d acc_raw = acc_B - grav_B + acc_corr;

  // オフセットが並進のみならばジャイロセンサの読みはベースフレームの角速度に一致する
  Vector3d gyro_raw = omega_B;

  // Add noise to the true values
  addNoise(acc_raw, gyro_raw, dt);

  // Update LPFs
  acc_lpf_.update(acc_raw, dt);
  gyro_lpf_.update(gyro_raw, dt);

  // Fill IMU message
  timeGazeboToRos(cur_time, imu_msg_.header.stamp);
  vectorGazeboToRos(acc_lpf_.getState(), imu_msg_.linear_acceleration);
  vectorGazeboToRos(gyro_lpf_.getState(), imu_msg_.angular_velocity);

  const double acc_var = sqr(acc_noise_density_obs_) / dt;
  fillMatrix3Diag(imu_msg_.linear_acceleration_covariance, acc_var);

  const double gyro_var = sqr(gyro_noise_density_obs_) / dt;
  fillMatrix3Diag(imu_msg_.angular_velocity_covariance, gyro_var);

  // Publish IMU message
  imu_pub_.publish(imu_msg_);

  // Fill and publish debug message
  debug_msg_.header = imu_msg_.header;
  vectorGazeboToKDL(acc_bias_, debug_msg_.acc_bias);
  vectorGazeboToKDL(gyro_bias_, debug_msg_.gyro_bias);
  debug_pub_.publish(debug_msg_);
}

void GazeboImuPlugin::addNoise(Vector3d& acc, Vector3d& gyro, const double& dt)
{
  // Gyrosocpe
  const auto tau_g = gyro_bias_corr_time_;
  // Discrete-time std. dev equivalent to an "integrating" sampler with integration time dt
  const auto sigma_g_d = gyro_noise_density_sig_ / sqrt(dt);  // [rad/s]
  const auto sigma_b_g = gyro_random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114] (memo: 2-32)
  const auto sigma_b_g_d = sigma_b_g * sqrt(tau_g / 2 * (1 - exp(-2 * dt / tau_g)));  // [rad/s]
  // Compute state-transition
  const auto phi_g_d = exp(-dt / tau_g);
  // Simulate gyroscope noise processes and add them to the true angular rate
  for (uint32_t i = 0; i < 3; ++i)
  {
    gyro_bias_[i] = phi_g_d * gyro_bias_[i] + sigma_b_g_d * noise_(rnd_gen_);
    gyro[i] += gyro_bias_[i] + sigma_g_d * noise_(rnd_gen_) + gyro_turn_on_bias_[i];
  }

  // Accelerometer
  const auto tau_a = acc_bias_corr_time_;
  // Discrete-time std. dev equivalent to an "integrating" sampler with integration time dt
  const auto sigma_a_d = acc_noise_density_sig_ / sqrt(dt);  // [m/s^2]
  const auto sigma_b_a = acc_random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114] (memo: 2-32)
  const auto sigma_b_a_d = sigma_b_a * sqrt(tau_a / 2 * (1 - exp(-2 * dt / tau_a)));  // [m/s^2]
  // Compute state-transition
  const auto phi_a_d = exp(-dt / tau_a);
  // Simulate accelerometer noise processes and add them to the true linear acceleration
  for (uint32_t i = 0; i < 3; ++i)
  {
    acc_bias_[i] = phi_a_d * acc_bias_[i] + sigma_b_a_d * noise_(rnd_gen_);
    acc[i] += acc_bias_[i] + sigma_a_d * noise_(rnd_gen_) + acc_turn_on_bias_[i];
  }
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboImuPlugin);
}  // namespace gazebo
