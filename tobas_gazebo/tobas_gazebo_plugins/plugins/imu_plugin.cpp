#include <tobas_math/core.hpp>
#include <tobas_msgs/Imu.hpp>
#include <tobas_gazebo_msgs/ImuDebug.h>

#include "./imu_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace std;
using namespace gz::math;

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
    gzthrow(kPluginName << ": Couldn't find specified link \"" << link_name_ << "\".");

  noise_ = NormalDistribution(0, 1);
  for (size_t i = 0; i < 3; ++i)
  {
    acc_turn_on_bias_[i] = acc_turn_on_bias_sigma_ * noise_(rnd_gen_);
    gyro_turn_on_bias_[i] = gyro_turn_on_bias_sigma_ * noise_(rnd_gen_);
  }

  // Initialize LPFs
  acc_lpf_.initialize(acc_lpf_cutoff_freq_, zero3);
  gyro_lpf_.initialize(gyro_lpf_cutoff_freq_, zero3);

  // Advertise
  imu_pub_ = createPublisher<tobas_msgs::Imu>("/" + ns() + "/" + tobas::kImuTopic);
  debug_pub_ = createPublisher<tobas_gazebo_msgs::ImuDebug>("/" + ns() + "/" + kDebugPubTopic);

  // Listen to the update event
  update_connection_ = sensor->ConnectUpdated(std::bind(&self::onUpdate, this));
}

void GazeboImuPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "robotNamespace", ns());
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "offset", offset_, zero3);

  getSdfParam(sdf, "accelNoiseDensityOnSignal", acc_noise_density_sig_, kDefaultAccNoiseDensity, POSITIVE);
  getSdfParam(sdf, "accelNoiseDensityObserved", acc_noise_density_obs_, kDefaultAccNoiseDensity, POSITIVE);
  getSdfParam(sdf, "accelRandomWalk", acc_random_walk_, kDefaultAccRandomWalk, POSITIVE);
  getSdfParam(sdf, "accelBiasCorrelationTime", acc_bias_corr_time_, kDefaultAccBiasCorrTime, POSITIVE);
  getSdfParam(sdf, "accelTurnOnBiasSigma", acc_turn_on_bias_sigma_, kDefaultAccTurnOnBiasSigma, POSITIVE);
  getSdfParam(sdf, "accelLpfCutoffFreq", acc_lpf_cutoff_freq_, kDefaultAccLpfCutoffFreq, POSITIVE);

  getSdfParam(sdf, "gyroNoiseDensityOnSignal", gyro_noise_density_sig_, kDefaultGyroNoiseDensity, POSITIVE);
  getSdfParam(sdf, "gyroNoiseDensityObserved", gyro_noise_density_obs_, kDefaultGyroNoiseDensity, POSITIVE);
  getSdfParam(sdf, "gyroRandomWalk", gyro_random_walk_, kDefaultGyroRandomWalk, POSITIVE);
  getSdfParam(sdf, "gyroBiasCorrelationTime", gyro_bias_corr_time_, kDefaultGyroBiasCorrTime, POSITIVE);
  getSdfParam(sdf, "gyroTurnOnBiasSigma", gyro_turn_on_bias_sigma_, kDefaultGyroTurnOnBiasSigma, POSITIVE);
  getSdfParam(sdf, "gyroLpfCutoffFreq", gyro_lpf_cutoff_freq_, kDefaultGyroLpfCutoffFreq, POSITIVE);
}

void GazeboImuPlugin::onUpdate()
{
  const auto cur_time = world_->SimTime();
  const auto dt = (cur_time - last_time_).Double();
  last_time_ = cur_time;

  // ベースフレームの状態を取得
  const auto& T_W_B = link_->WorldPose();
  const auto& R_W_B = T_W_B.Rot();
  const auto acc_B = link_->RelativeLinearAccel();
  const auto omega_B = link_->RelativeAngularVel();
  const auto domega_B = link_->RelativeAngularAccel();

  // オフセットによる補正を考慮して加速度センサの読みを計算 (memo: 2-26)
  const auto grav_B = R_W_B.RotateVectorReverse(world_->Gravity());
  const auto acc_corr = omega_B.Cross(omega_B.Cross(offset_)) + domega_B.Cross(offset_);
  auto acc_raw = acc_B - grav_B + acc_corr;

  // オフセットが並進のみならばジャイロセンサの読みはベースフレームの角速度に一致する
  auto gyro_raw = omega_B;

  // Add noise to the true values
  addNoise(acc_raw, gyro_raw, dt);

  // Update LPFs
  acc_lpf_.update(acc_raw, dt);
  gyro_lpf_.update(gyro_raw, dt);

  publishImuMsg(dt);
  publishDebugMsg();
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
  for (size_t i = 0; i < 3; ++i)
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
  for (size_t i = 0; i < 3; ++i)
  {
    acc_bias_[i] = phi_a_d * acc_bias_[i] + sigma_b_a_d * noise_(rnd_gen_);
    acc[i] += acc_bias_[i] + sigma_a_d * noise_(rnd_gen_) + acc_turn_on_bias_[i];
  }
}

void GazeboImuPlugin::publishImuMsg(const double& dt) const
{
  const auto imu_msg =std::make_unique<tobas_msgs::Imu>();

  timeGazeboToRos(world_->SimTime(), imu_msg->header.stamp);
  imu_msg->header.frame_id = link_name_;

  vectorGazeboToKDL(acc_lpf_.getOutput(), imu_msg->accel);
  const auto acc_var = math::sqr(acc_noise_density_obs_) / dt;
  imu_msg->accel_covariance = Eigen::Vector3d::Constant(acc_var).asDiagonal();

  vectorGazeboToKDL(gyro_lpf_.getOutput(), imu_msg->gyro);
  const auto gyro_var = math::sqr(gyro_noise_density_obs_) / dt;
  imu_msg->gyro_covariance = Eigen::Vector3d::Constant(gyro_var).asDiagonal();

  imu_pub_->publish(imu_msg);
}

void GazeboImuPlugin::publishDebugMsg() const
{
  const auto debug_msg =std::make_unique<tobas_gazebo_msgs::ImuDebug>();

  timeGazeboToRos(world_->SimTime(), debug_msg->header.stamp);
  debug_msg->header.frame_id = link_name_;

  vectorGazeboToKDL(acc_bias_, debug_msg->acc_bias);
  vectorGazeboToKDL(gyro_bias_, debug_msg->gyro_bias);

  debug_pub_->publish(debug_msg);
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboImuPlugin);
}  // namespace gazebo
