#include <tobas_constants/constants.hpp>
#include <tobas_dsp/low_pass_filter_p1.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/model_mass_holder.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_math/core.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/universal_constants.hpp>

#include <tobas_gazebo_msgs/msg/engine_state.hpp>
#include <tobas_gazebo_msgs/msg/imu_debug.hpp>
#include <tobas_gazebo_msgs/msg/rotor_state.hpp>
#include <tobas_msgs/srv/configure_imu_filter.hpp>
#include <tobas_msgs_adapter/imu.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/conversions/conversions.hpp"
#include "tobas_gazebo_system_plugins/random.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
/**
 * @brief Gazebo IMU plugin
 *
 * - 初期バイアスはキャリブレーション済みの想定．
 */
class GazeboImuPlugin : public BaseNode,
                        public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPostUpdate
{
  // Constants
  static constexpr char kDebugPubTopic[] = "gazebo/imu_debug";

  // TODO: 加速度の比率やジャイロの振動も真面目に考察
  static constexpr double kVibrationAccVerHorRate = 1.;
  static constexpr double kVibrationAccGyroRate = 0.05;
  static constexpr double kVibrationGyroAttiHeadRate = 0.5;

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
  string link_name_;
  size_t update_rate_;          // Update rate [Hz]
  gz::math::Vector3d offset_;   // B_Pos_BS
  double acc_noise_density_;    // Accel noise density [m/s^2/√Hz]
  double acc_random_walk_;      // Accel bias random walk [m/s^2/s/√Hz]
  double acc_bias_corr_time_;   // Accel bias correlation time constant [s]
  double gyro_noise_density_;   // Gyro noise density [rad/s/√Hz]
  double gyro_random_walk_;     // Gyro bias random walk [rad/s/s/√Hz]
  double gyro_bias_corr_time_;  // Gyro bias correlation time constant [s]
  vector<string> rotor_link_names_;

  const cmp::WorldPose* pose_W_;
  const cmp::LinearAcceleration* acc_B_;
  const cmp::AngularVelocity* gyro_B_;
  const cmp::AngularAcceleration* dgyro_B_;
  const cmp::Gravity* grav_W_;

  RateManager::SharedPtr rate_manager_;
  ModelMassHolder mass_holder_;
  dsp::LowPassFilterP1<gz::math::Vector3d> acc_lpf_, gyro_lpf_, dgyro_lpf_;
  bool lpf_initialized_ = false;
  gz::math::Vector3d acc_bias_ = gz::math::Vector3d::Zero;
  gz::math::Vector3d gyro_bias_ = gz::math::Vector3d::Zero;
  gz::math::Vector3d prev_gyro_meas_ = gz::math::Vector3d::Zero;
  double engine_vibration_force_;               // [N] エンジンで発生する振動力
  map<string, double> rotor_vibration_forces_;  // [N] 各モータで発生する振動力

  random_device rnd_dev_;
  NormalDistribution3d normal_;

  ros2::PublisherPtr<tobas_msgs::Imu> imu_raw_pub_;
  ros2::PublisherPtr<tobas_msgs::Imu> imu_filt_pub_;
  ros2::PublisherPtr<tobas_gazebo_msgs::msg::ImuDebug> debug_pub_;

  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::EngineState> engine_state_sub_;
  vector<ros2::SubscriberPtr<tobas_gazebo_msgs::msg::RotorState>> rotor_state_subs_;

  ros2::ServiceServerPtr<tobas_msgs::srv::ConfigureImuFilter> config_imu_filter_srv_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void addNoise(gz::math::Vector3d& acc, gz::math::Vector3d& gyro, const double& dt);

  void engineStateCb(const tobas_gazebo_msgs::msg::EngineState::ConstSharedPtr& msg);

  void configureImuFilterCb(
    const tobas_msgs::srv::ConfigureImuFilter::Request::ConstSharedPtr& req,
    const tobas_msgs::srv::ConfigureImuFilter::Response::SharedPtr& res);
};

GazeboImuPlugin::GazeboImuPlugin() : normal_(rnd_dev_, 0., 1.)
{
  acc_lpf_.setValue(gz::math::Vector3d::Zero);
  gyro_lpf_.setValue(gz::math::Vector3d::Zero);
  dgyro_lpf_.setValue(gz::math::Vector3d::Zero);

  // TODO: 消す
  acc_lpf_.setCutoffFrequency(30);
  gyro_lpf_.setCutoffFrequency(40);
  dgyro_lpf_.setCutoffFrequency(20);
  lpf_initialized_ = true;
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

  if (!mass_holder_.initialize(model, ecm)) {
    TOBAS_EXIT("Failed to initialize model mass holder.");
  }

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");
  }

  const auto world = ecm.EntityByComponents(cmp::World());
  if (world == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to get the world component.");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  acc_B_ = getComponent<cmp::LinearAcceleration>(link, ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link, ecm);
  dgyro_B_ = getComponent<cmp::AngularAcceleration>(link, ecm);
  grav_W_ = getComponent<cmp::Gravity>(world, ecm);

  imu_raw_pub_ = createPublisher<tobas_msgs::Imu>(tobas::kImuRawTopic);
  imu_filt_pub_ = createPublisher<tobas_msgs::Imu>(tobas::kImuFiltTopic);
  debug_pub_ = createPublisher<tobas_gazebo_msgs::msg::ImuDebug>(kDebugPubTopic);

  engine_state_sub_ = createSubscriber(kEngineStateGtTopic, &self::engineStateCb, this);

  // モータ状態のコールバックとサブスクライバを設定
  for (const auto& link_name : rotor_link_names_) {
    const auto topic = path::join(kRotorStateGtTopicNS, link_name);
    const auto qos = ros2::makeQoS(false, false, 1);
    const auto cb = [this, link_name](const tobas_gazebo_msgs::msg::RotorState::ConstSharedPtr& msg)
    { rotor_vibration_forces_[link_name] = msg->vibration_force; };
    const auto sub = node_->create_subscription<tobas_gazebo_msgs::msg::RotorState>(topic, qos, cb);
    rotor_state_subs_.push_back(sub);
  }

  config_imu_filter_srv_ = createService<tobas_msgs::srv::ConfigureImuFilter>(
    tobas::kConfigureImuFilterSrv, &self::configureImuFilterCb, this);
}

void GazeboImuPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, NON_NEGATIVE);
  getSdfParam(sdf, "offset", offset_);

  getSdfParam(sdf, "accelNoiseDensity", acc_noise_density_, NON_NEGATIVE);
  getSdfParam(sdf, "accelRandomWalk", acc_random_walk_, NON_NEGATIVE);
  getSdfParam(sdf, "accelBiasCorrelationTime", acc_bias_corr_time_, POSITIVE);

  getSdfParam(sdf, "gyroNoiseDensity", gyro_noise_density_, NON_NEGATIVE);
  getSdfParam(sdf, "gyroRandomWalk", gyro_random_walk_, NON_NEGATIVE);
  getSdfParam(sdf, "gyroBiasCorrelationTime", gyro_bias_corr_time_, POSITIVE);

  getSdfParam(sdf, "rotorLinkNames", rotor_link_names_);
}

void GazeboImuPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (rotor_vibration_forces_.size() < rotor_link_names_.size()) {
    if (info.simTime > kWarnStartTime) {
      const auto num_not_received = rotor_link_names_.size() - rotor_vibration_forces_.size();
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

  // Compute D-gyro
  const auto dgyro_meas = (gyro_meas - prev_gyro_meas_) / dt;
  prev_gyro_meas_ = gyro_meas;

  // Filter
  if (lpf_initialized_) {
    acc_lpf_.update(acc_meas, dt);
    gyro_lpf_.update(gyro_meas, dt);
    dgyro_lpf_.update(dgyro_meas, dt);
  }

  // Publish rate filter
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // Publish raw IMU message
  auto imu_raw_msg = make_unique<tobas_msgs::Imu>();
  ros2::timeChronoToMsg(info.simTime, imu_raw_msg->header.stamp);
  imu_raw_msg->header.frame_id = link_name_;
  vectorGazeboToKDL(acc_meas, imu_raw_msg->accel);
  vectorGazeboToKDL(gyro_meas, imu_raw_msg->gyro);
  vectorGazeboToKDL(dgyro_meas, imu_raw_msg->dgyro);
  imu_raw_pub_->publish(move(imu_raw_msg));

  // Publish filtered IMU message
  if (lpf_initialized_) {
    auto imu_filt_msg = make_unique<tobas_msgs::Imu>();
    ros2::timeChronoToMsg(info.simTime, imu_filt_msg->header.stamp);
    imu_filt_msg->header.frame_id = link_name_;
    vectorGazeboToKDL(acc_lpf_.getValue(), imu_filt_msg->accel);
    vectorGazeboToKDL(gyro_lpf_.getValue(), imu_filt_msg->gyro);
    vectorGazeboToKDL(dgyro_lpf_.getValue(), imu_filt_msg->dgyro);
    imu_filt_pub_->publish(move(imu_filt_msg));
  }

  // Publish debug message
  auto debug_msg = make_unique<tobas_gazebo_msgs::msg::ImuDebug>();
  ros2::timeChronoToMsg(info.simTime, debug_msg->header.stamp);
  debug_msg->header.frame_id = link_name_;
  vectorGazeboToMsg(acc_bias_, debug_msg->acc_bias);
  vectorGazeboToMsg(gyro_bias_, debug_msg->gyro_bias);
  debug_pub_->publish(move(debug_msg));
}

void GazeboImuPlugin::addNoise(gz::math::Vector3d& acc, gz::math::Vector3d& gyro, const double& dt)
{
  // Compute vibration force
  auto vibration_force_sum = engine_vibration_force_;
  for (const auto& [_, vibration_force] : rotor_vibration_forces_) {
    vibration_force_sum += vibration_force;
  }
  const auto vibration_acc_ver = vibration_force_sum / mass_holder_.getMass();        // [m/s^2]
  const auto vibration_acc_hor = vibration_acc_ver * kVibrationAccVerHorRate;         // [m/s^2]
  const auto vibration_gyro_atti = vibration_acc_ver * kVibrationAccGyroRate;         // [rad/s]
  const auto vibration_gyro_head = vibration_gyro_atti * kVibrationGyroAttiHeadRate;  // [rad/s]
  const gz::math::Vector3d vibration_acc(vibration_acc_hor, vibration_acc_hor, vibration_acc_ver);
  const gz::math::Vector3d vibration_gyro(vibration_gyro_atti, vibration_gyro_atti, vibration_gyro_head);

  // Accel
  const auto tau_a = acc_bias_corr_time_;
  // Discrete-time std. dev equivalent to an "integrating" sampler with integration time dt
  const auto sigma_a_d = acc_noise_density_ / sqrt(dt);  // [m/s^2]
  const auto sigma_b_a = acc_random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114] (memo: 2-32)
  const auto sigma_b_a_d = sigma_b_a * sqrt(tau_a / 2 * (1 - exp(-2 * dt / tau_a)));  // [m/s^2]
  // Compute state-transition
  const auto phi_a_d = exp(-dt / tau_a);
  // Simulate accelerometer noise processes and add them to the true linear acceleration
  acc_bias_ = phi_a_d * acc_bias_ + sigma_b_a_d * normal_.get();
  acc += sigma_a_d * normal_.get() + acc_bias_ + vibration_acc;

  // Gyro
  const auto tau_g = gyro_bias_corr_time_;
  // Discrete-time std. dev equivalent to an "integrating" sampler with integration time dt
  const auto sigma_g_d = gyro_noise_density_ / sqrt(dt);  // [rad/s]
  const auto sigma_b_g = gyro_random_walk_;
  // Compute exact covariance of the process after dt [Maybeck 4-114] (memo: 2-32)
  const auto sigma_b_g_d = sigma_b_g * sqrt(tau_g / 2 * (1 - exp(-2 * dt / tau_g)));  // [rad/s]
  // Compute state-transition
  const auto phi_g_d = exp(-dt / tau_g);
  // Simulate gyroscope noise processes and add them to the true angular rate
  gyro_bias_ = phi_g_d * gyro_bias_ + sigma_b_g_d * normal_.get();
  gyro += sigma_g_d * normal_.get() + gyro_bias_ + vibration_gyro;
}

void GazeboImuPlugin::engineStateCb(const tobas_gazebo_msgs::msg::EngineState::ConstSharedPtr& msg)
{
  engine_vibration_force_ = msg->vibration_force;
}

void GazeboImuPlugin::configureImuFilterCb(
  const tobas_msgs::srv::ConfigureImuFilter::Request::ConstSharedPtr& req,
  const tobas_msgs::srv::ConfigureImuFilter::Response::SharedPtr& res)
{
  if (!acc_lpf_.setCutoffFrequency(req->accel_cutoff)) {
    res->success = false;
    res->message = "Failed to set accel LPF cutoff frequency.";
    return;
  }

  if (!gyro_lpf_.setCutoffFrequency(req->gyro_cutoff)) {
    res->success = false;
    res->message = "Failed to set gyro LPF cutoff frequency.";
    return;
  }

  if (!dgyro_lpf_.setCutoffFrequency(req->dgyro_cutoff)) {
    res->success = false;
    res->message = "Failed to set D-gyro LPF cutoff frequency.";
    return;
  }

  res->success = true;
  res->message.clear();
}
}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::GazeboImuPlugin,
  gz::sim::System,
  gazebo::GazeboImuPlugin::ISystemConfigure,
  gazebo::GazeboImuPlugin::ISystemPostUpdate)
