#include "../../include/multirotor_gazebo_plugins/imu_plugin.hpp"
#include "../../include/multirotor_gazebo_plugins/utils.hpp"

#define GRAVITY 9.80665
#define ZERO_3 (ignition::math::Vector3d(0., 0., 0.))

using namespace std;

namespace gazebo
{
GazeboImuPlugin::GazeboImuPlugin()
  : ModelPlugin(), rnd_gen_(rnd_dev_()), velocity_prev_W_(0., 0., 0.)
{
}

void GazeboImuPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Get SDF parameters
  getSdfParams(sdf);

  // Store the pointer to the model
  model_ = model;
  world_ = model_->GetWorld();

  // Get the pointer to the link
  link_ = model_->GetLink(link_name_);
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

  // Advertise publisher
  imu_pub_ = nh_.advertise<ImuMsg>("/" + ns_ + "/" + imu_topic_, 1);

  // Listen to the update event
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboImuPlugin::onUpdate, this, _1));
}

void GazeboImuPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  if (sdf->HasElement("robotNamespace"))
  {
    ns_ = sdf->GetElement("robotNamespace")->Get<string>();
  }
  else
  {
    gzthrow(kPluginName << ": Please specify a robotNamespace.");
  }

  if (sdf->HasElement("linkName"))
  {
    link_name_ = sdf->GetElement("linkName")->Get<string>();
  }
  else
  {
    gzthrow(kPluginName << ": Please specify a linkName.");
  }

  // TODO: 範囲チェック
  getSdfParam<string>(sdf, "imuTopic", imu_topic_, kDefaultImuTopic);
  getSdfParam<double>(sdf, "gyroscopeNoiseDensity", gyro_noise_density_, kDefaultGyroNoiseDensity);
  getSdfParam<double>(sdf, "gyroscopeBiasRandomWalk", gyro_random_walk_, kDefaultGyroRandomWalk);
  getSdfParam<double>(
    sdf, "gyroscopeBiasCorrelationTime", gyro_bias_corr_time_, kDefaultGyroBiasCorrTime);
  getSdfParam<double>(
    sdf, "gyroscopeTurnOnBiasSigma", gyro_turn_on_bias_sigma_, kDefaultGyroTurnOnBiasSigma);
  getSdfParam<double>(
    sdf, "accelerometerNoiseDensity", acc_noise_density_, kDefaultAccNoiseDensity);
  getSdfParam<double>(sdf, "accelerometerRandomWalk", acc_random_walk_, kDefaultAccRandomWalk);
  getSdfParam<double>(
    sdf, "accelerometerBiasCorrelationTime", acc_bias_corr_time_, kDefaultAccBiasCorrTime);
  getSdfParam<double>(
    sdf, "accelerometerTurnOnBiasSigma", acc_turn_on_bias_sigma_, kDefaultAccTurnOnBiasSigma);
}

void GazeboImuPlugin::addNoise(
  ignition::math::Vector3d& lin_acc,
  ignition::math::Vector3d& ang_vel,
  double dt)
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

void GazeboImuPlugin::onUpdate(const common::UpdateInfo&)
{
  common::Time cur_time = world_->SimTime();
  double dt = (cur_time - last_time_).Double();
  GZ_ASSERT(dt > 0., "Change in time must be greater than 0.");
  last_time_ = cur_time;

  // Get linear acceleration and angular velocity from simulation
  ignition::math::Pose3d T_W_B = link_->WorldPose();
  ignition::math::Quaterniond R_W_B = T_W_B.Rot();
  ignition::math::Vector3d acc_B =
    link_->RelativeLinearAccel() - R_W_B.RotateVectorReverse(gravity_W_);
  ignition::math::Vector3d angvel_B = link_->RelativeAngularVel();

  // Add noise to the true values
  addNoise(acc_B, angvel_B, dt);

  // Fill IMU message.
  imu_msg_.header.stamp.sec = cur_time.sec;
  imu_msg_.header.stamp.nsec = cur_time.nsec;

  imu_msg_.linear_acceleration.x = acc_B[0];
  imu_msg_.linear_acceleration.y = acc_B[1];
  imu_msg_.linear_acceleration.z = acc_B[2];

  imu_msg_.angular_velocity.x = angvel_B[0];
  imu_msg_.angular_velocity.y = angvel_B[1];
  imu_msg_.angular_velocity.z = angvel_B[2];

  double acc_var = sqr(acc_noise_density_) / dt;
  imu_msg_.linear_acceleration_covariance[0] = acc_var;
  imu_msg_.linear_acceleration_covariance[4] = acc_var;
  imu_msg_.linear_acceleration_covariance[8] = acc_var;

  double angvel_var = sqr(gyro_noise_density_) / dt;
  imu_msg_.angular_velocity_covariance[0] = angvel_var;
  imu_msg_.angular_velocity_covariance[4] = angvel_var;
  imu_msg_.angular_velocity_covariance[8] = angvel_var;

  // Publish IMU message
  imu_pub_.publish(imu_msg_);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboImuPlugin);
}  // namespace gazebo
