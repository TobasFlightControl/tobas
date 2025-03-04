#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/model_mass_holder.hpp>
#include <tobas_gazebo_msgs/msg/imu_debug.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>

#include "../include/tobas_gazebo_plugins/common/common.hpp"
#include "../include/tobas_gazebo_plugins/conversions/conversions.hpp"
#include "../include/tobas_gazebo_plugins/rate_manager.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
class GazeboImuPlugin : public BaseNode,
                        public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPostUpdate
{
  // Constants
  static constexpr char kDebugPubTopic[] = "gazebo/imu_debug";
  static constexpr double kAccGyroRotorNoiseRate = 0.1;  // TODO: モータのジャイロへの影響も真面目に考察

  // Default values
  static constexpr size_t kDefaultUpdateRate = 400;  // [Hz]
  static constexpr double kDefaultAccNoiseDensity = 2. * 2e-3;
  static constexpr double kDefaultAccRandomWalk = 2. * 3e-3;
  static constexpr double kDefaultAccBiasCorrTime = 300.;
  static constexpr double kDefaultAccTurnOnBiasSigma = 2e-2 * tobas_std::kGravity;
  static constexpr double kDefaultGyroNoiseDensity = 2. * 35. / 3600. * tobas_std::kDeg2Rad;
  static constexpr double kDefaultGyroRandomWalk = 2. * 4. / 3600. * tobas_std::kDeg2Rad;
  static constexpr double kDefaultGyroBiasCorrTime = 1000.;
  static constexpr double kDefaultGyroTurnOnBiasSigma = 0.5 * tobas_std::kDeg2Rad;

  using self = GazeboImuPlugin;

public:
  explicit GazeboImuPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string link_name_;
  size_t update_rate_;              // Update rate [Hz]
  gz::math::Vector3d offset_;       // B_Pos_BS
  double acc_noise_density_sig_;    // Accel noise density actually added to signal [m/s^2/√Hz]
  double acc_noise_density_obs_;    // Accel noise density that is observerd [m/s^2/√Hz]
  double acc_random_walk_;          // Accel bias random walk [m/s^2/s/√Hz]
  double acc_bias_corr_time_;       // Accel bias correlation time constant [s]
  double acc_turn_on_bias_sigma_;   // Accel turn on bias standard deviation [m/s^2]
  double gyro_noise_density_sig_;   // Gyro noise density actually added to signal [rad/s/√Hz]
  double gyro_noise_density_obs_;   // Gyro noise density that is observed [rad/s/√Hz]
  double gyro_random_walk_;         // Gyro bias random walk [rad/s/s/√Hz]
  double gyro_bias_corr_time_;      // Gyro bias correlation time constant [s]
  double gyro_turn_on_bias_sigma_;  // Gyro turn on bias standard deviation [rad/s]
  vector<string> rotor_link_names_;

  const cmp::WorldPose* pose_W_;
  const cmp::LinearAcceleration* acc_B_;
  const cmp::AngularVelocity* gyro_B_;
  const cmp::AngularAcceleration* dgyro_B_;
  const cmp::Gravity* grav_W_;

  RateManager::SharedPtr rate_manager_;
  ModelMassHolder mass_holder_;
  gz::math::Vector3d acc_bias_ = gz::math::Vector3d::Zero;
  gz::math::Vector3d gyro_bias_ = gz::math::Vector3d::Zero;
  gz::math::Vector3d acc_turn_on_bias_;
  gz::math::Vector3d gyro_turn_on_bias_;
  map<string, double> rotor_noises_;  // [N] 各モータで発生する周波数ノイズ

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  NormalDistribution noise_;

  ros2::PublisherPtr<tobas_msgs::ImuStamped> imu_pub_;
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::ImuDebug> debug_pub_;
  vector<ros2::SubscriberPtr<tobas_gazebo_msgs::msg::RotorState>> rotor_state_subs_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void addNoise(gz::math::Vector3d& acc, gz::math::Vector3d& gyro, const double& dt);
};

GazeboImuPlugin::GazeboImuPlugin() : rnd_gen_(rnd_dev_())
{
}

void GazeboImuPlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_imu_plugin", sdf);
  getSdfParams(sdf);

  rate_manager_ = make_shared<RateManager>(update_rate_);

  if (!mass_holder_.initialize(model, ecm))
    TOBAS_EXIT("Failed to initialize model mass holder.");

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity)
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");

  const auto world = ecm.EntityByComponents(cmp::World());
  if (world == gz::sim::kNullEntity)
    TOBAS_EXIT("Failed to get the world component.");

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  acc_B_ = getComponent<cmp::LinearAcceleration>(link, ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link, ecm);
  dgyro_B_ = getComponent<cmp::AngularAcceleration>(link, ecm);
  grav_W_ = getComponent<cmp::Gravity>(world, ecm);

  noise_ = NormalDistribution(0, 1);
  for (size_t i = 0; i < 3; ++i)
  {
    acc_turn_on_bias_[i] = acc_turn_on_bias_sigma_ * noise_(rnd_gen_);
    gyro_turn_on_bias_[i] = gyro_turn_on_bias_sigma_ * noise_(rnd_gen_);
  }

  imu_pub_ = createPublisher<tobas_msgs::ImuStamped>(tobas::kImuRawTopic);
  debug_pub_ = createPublisher<tobas_gazebo_msgs::msg::ImuDebug>(kDebugPubTopic);

  // モータ状態のコールバックとサブスクライバを設定
  for (const auto& link_name : rotor_link_names_)
  {
    const auto topic = path::join(kRotorStateGtTopicNS, link_name);
    const auto qos = ros2::makeQoS(false, false, 1);
    const auto cb = [this, link_name](const tobas_gazebo_msgs::msg::RotorState::ConstSharedPtr& msg)
    { rotor_noises_[link_name] = msg->rotor_noise; };
    const auto sub = node_->create_subscription<tobas_gazebo_msgs::msg::RotorState>(topic, qos, cb);
    rotor_state_subs_.push_back(sub);
  }
}

void GazeboImuPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, NON_NEGATIVE);
  getSdfParam(sdf, "offset", offset_, gz::math::Vector3d::Zero);

  getSdfParam(sdf, "accelNoiseDensityOnSignal", acc_noise_density_sig_, kDefaultAccNoiseDensity, NON_NEGATIVE);
  getSdfParam(sdf, "accelNoiseDensityObserved", acc_noise_density_obs_, kDefaultAccNoiseDensity, NON_NEGATIVE);
  getSdfParam(sdf, "accelRandomWalk", acc_random_walk_, kDefaultAccRandomWalk, NON_NEGATIVE);
  getSdfParam(sdf, "accelBiasCorrelationTime", acc_bias_corr_time_, kDefaultAccBiasCorrTime, POSITIVE);
  getSdfParam(sdf, "accelTurnOnBiasSigma", acc_turn_on_bias_sigma_, kDefaultAccTurnOnBiasSigma, NON_NEGATIVE);

  getSdfParam(sdf, "gyroNoiseDensityOnSignal", gyro_noise_density_sig_, kDefaultGyroNoiseDensity, NON_NEGATIVE);
  getSdfParam(sdf, "gyroNoiseDensityObserved", gyro_noise_density_obs_, kDefaultGyroNoiseDensity, NON_NEGATIVE);
  getSdfParam(sdf, "gyroRandomWalk", gyro_random_walk_, kDefaultGyroRandomWalk, NON_NEGATIVE);
  getSdfParam(sdf, "gyroBiasCorrelationTime", gyro_bias_corr_time_, kDefaultGyroBiasCorrTime, POSITIVE);
  getSdfParam(sdf, "gyroTurnOnBiasSigma", gyro_turn_on_bias_sigma_, kDefaultGyroTurnOnBiasSigma, NON_NEGATIVE);

  getSdfParam(sdf, "rotorLinkNames", rotor_link_names_);
}

void GazeboImuPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime))
    return;

  if (rotor_noises_.size() < rotor_link_names_.size())
  {
    if (info.simTime > kWarnStartTime)
    {
      const auto num_not_received = rotor_link_names_.size() - rotor_noises_.size();
      TOBAS_WARN_THROTTLE(kWarnPeriod, to_string(num_not_received), " rotor states are not received yet.");
    }
  }

  // ベースフレームの状態を取得
  const auto& T_W_B = pose_W_->Data();
  const auto& R_W_B = T_W_B.Rot();
  const auto& acc_B = acc_B_->Data();
  const auto& gyro_B = gyro_B_->Data();
  const auto& dgyro_B = dgyro_B_->Data();

  // オフセットによる補正を考慮して加速度センサの読みを計算 (memo: 2-26)
  const auto grav_B = R_W_B.RotateVectorReverse(grav_W_->Data());
  const auto acc_corr = gyro_B.Cross(gyro_B.Cross(offset_)) + dgyro_B.Cross(offset_);
  auto acc_meas = acc_B - grav_B + acc_corr;

  // オフセットが並進のみならばジャイロセンサの読みはベースフレームの角速度に一致する
  auto gyro_meas = gyro_B;

  // Get delta time
  const auto dt = chrono::duration<double>(info.dt).count();

  // Add noise to the true values
  addNoise(acc_meas, gyro_meas, dt);

  // Publish IMU message
  auto imu_msg = std::make_unique<tobas_msgs::ImuStamped>();
  ros2::timeChronoToMsg(info.simTime, imu_msg->header.stamp);
  imu_msg->header.frame_id = link_name_;
  vectorGazeboToKDL(acc_meas, imu_msg->imu.accel);
  vectorGazeboToKDL(gyro_meas, imu_msg->imu.gyro);
  imu_pub_->publish(move(imu_msg));

  // Publish debug message
  auto debug_msg = std::make_unique<tobas_gazebo_msgs::msg::ImuDebug>();
  ros2::timeChronoToMsg(info.simTime, debug_msg->header.stamp);
  debug_msg->header.frame_id = link_name_;
  vectorGazeboToMsg(acc_bias_, debug_msg->acc_bias);
  vectorGazeboToMsg(gyro_bias_, debug_msg->gyro_bias);
  debug_pub_->publish(move(debug_msg));
}

void GazeboImuPlugin::addNoise(gz::math::Vector3d& acc, gz::math::Vector3d& gyro, const double& dt)
{
  // Compute rotor noise
  double rotor_noise_sum = 0;
  for (const auto& [_, rotor_noise] : rotor_noises_)
    rotor_noise_sum += rotor_noise;
  const auto rotor_noise_acc = rotor_noise_sum / mass_holder_.getMass();
  const auto rotor_noise_gyro = rotor_noise_acc * kAccGyroRotorNoiseRate;

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
    acc[i] += sigma_a_d * noise_(rnd_gen_) + acc_bias_[i] + acc_turn_on_bias_[i] + rotor_noise_acc;
  }

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
    gyro[i] += sigma_g_d * noise_(rnd_gen_) + gyro_bias_[i] + gyro_turn_on_bias_[i] + rotor_noise_gyro;
  }
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboImuPlugin,
  gz::sim::System,
  gazebo::GazeboImuPlugin::ISystemConfigure,
  gazebo::GazeboImuPlugin::ISystemPostUpdate)
