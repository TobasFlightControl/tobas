#include <bit>

#include <gz/transport/Node.hh>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tobas_constants/constants.hpp>
#include <tobas_gazebo_system_plugins/common/common.hpp>
#include <tobas_gazebo_system_plugins/random.hpp>
#include <tobas_gazebo_system_plugins/rate_manager.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>

namespace cmp = gz::sim::components;

namespace gazebo
{
/* gazeboのgpu lidarをros2でhesai lidarっぽく使用できるようにbridgeする．
   gpu_lidar sensorとgz-sim-sensors-system pluginにより発出されるgz::msgs::PointCloudPacked型のmessageをsubscribeして，
   ros2のsensor_msgs::msg::PointCloud2型のmessageとして再publishする．
   lidarにはImuもついているので，それも模擬する．こちらの実装は簡素化しているもののimu_plugin.hppに則る */
class HesaiLidarPlugin : public BaseNode,
                         public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPostUpdate
{
public:
  explicit HesaiLidarPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_mgr) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  static constexpr double kStaticAccThresh = 1.;    // [m/s^2]
  static constexpr double kStaticGyroThresh = 0.1;  // [rad/s]
  static constexpr bool kHesaiLatch = true;
  static constexpr bool kHesaiReliable = true;
  static constexpr int kHesaiQueueSize = 10;

  struct LidarParams
  {
    std::string gpu_ray_topic;
    int gpu_ray_update_rate;
    int update_rate;
    int horizontal_samples;
    int vertical_samples;
  } lidar_params_;

  struct ImuParams
  {
    std::string link_name;
    int update_rate;
    gz::math::Vector3d offset;
    double acc_noise_density;    // Accel noise density [m/s^2/√Hz]
    double acc_random_walk;      // Accel bias random walk [m/s^2/s/√Hz]
    double acc_bias_corr_time;   // Accel bias correlation time constant [s]
    double gyro_noise_density;   // Gyro noise density [rad/s/√Hz]
    double gyro_random_walk;     // Gyro bias random walk [rad/s/s/√Hz]
    double gyro_bias_corr_time;  // Gyro bias correlation time constant [s]
  } imu_params_;

  // lidar関連
  // 1回publishするまでに何回gpu_rayをsamplingするか
  int sampling_times_;
  // gpu_lidarを前のpublishから数えて何回受信して詰めたか
  int phase_ = 0;
  // messageに入っている1つの点の情報にどれくらいのサイズが使われているか
  uint32_t point_step_;
  // messageのデータ構造
  uint32_t x_offset_;
  uint32_t y_offset_;
  uint32_t z_offset_;
  uint32_t intensity_offset_;
  uint32_t ring_offset_;
  uint32_t timestamp_offset_;
  // 各ringに対して, 水平方向のsamplingを始めるindex
  // 点群データを歪ませるためにgpu_rayのデータからこのindexから開始した点のみを間引いてsamplingする
  std::vector<uint> sampling_start_idx_;
  // lidarのデータのどのindexのところにデータをつめるか
  uint lidar_data_index_ = 0;

  // imu関連
  RateManager::SharedPtr imu_rate_manager_;
  cmp::WorldPose* pose_W_;
  cmp::LinearAcceleration* acc_B_;
  cmp::AngularVelocity* gyro_B_;
  cmp::AngularAcceleration* dgyro_B_;
  cmp::Gravity* grav_W_;
  bool stationary_state_detected_ = false;
  gz::math::Vector3d acc_bias_ = gz::math::Vector3d::Zero;
  gz::math::Vector3d gyro_bias_ = gz::math::Vector3d::Zero;
  // In order to generate IMU noise
  std::random_device rnd_dev_;
  NormalDistribution3d normal_;

  // gazebo interfaces
  gz::transport::Node node_;

  // ros2 interfaces
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud2> lidar_point_cloud_publisher_;
  ros2::PublisherPtr<sensor_msgs::msg::Imu> imu_raw_pub_;
  sensor_msgs::msg::PointCloud2::UniquePtr point_cloud_msg_;

  // load configurations from sdf
  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void getLidarSdfParams(const sdf::ElementConstPtr& sdf);
  void getImuSdfParams(const sdf::ElementConstPtr& sdf);

  // lidar related functions
  void gpuRayCb(const gz::msgs::PointCloudPacked& msg);
  void setupPointCloudMsg(const gz::msgs::PointCloudPacked& msg);

  // imu related functions
  void addImuNoise(gz::math::Vector3d& acc, gz::math::Vector3d& gyro, const double& dt);

  // utility functions
  uint64_t convertToNanoSecond(const int64_t& sec, const int32_t& nanosec);
  void vectorGazeboToGeometryMsg(const gz::math::Vector3d& gazebo_msg, geometry_msgs::msg::Vector3& msg);
};

HesaiLidarPlugin::HesaiLidarPlugin() : normal_(rnd_dev_, 0., 1.)
{
}

void HesaiLidarPlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("hesai_lidar_plugin", sdf);
  getSdfParams(sdf);

  // imu related operations
  imu_rate_manager_ = std::make_shared<RateManager>(imu_params_.update_rate);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(imu_params_.link_name));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", imu_params_.link_name, "\".");
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

  // setup useful parameters
  x_offset_ = 0;
  y_offset_ = x_offset_ + sizeof(float);
  z_offset_ = y_offset_ + sizeof(float);
  intensity_offset_ = z_offset_ + sizeof(float);
  ring_offset_ = intensity_offset_ + sizeof(float);
  timestamp_offset_ = ring_offset_ + sizeof(uint16_t);
  point_step_ = timestamp_offset_ + sizeof(double);
  sampling_start_idx_.resize(lidar_params_.vertical_samples);
  for (size_t i = 0; i < sampling_start_idx_.size(); i++) {
    sampling_start_idx_[i] = std::floor(
      i * static_cast<float>(lidar_params_.horizontal_samples) / static_cast<float>(lidar_params_.vertical_samples));
  }

  // gazebo interfaces
  node_.Subscribe(lidar_params_.gpu_ray_topic, &HesaiLidarPlugin::gpuRayCb, this);

  // ros interfaces
  lidar_point_cloud_publisher_ = createPublisher<sensor_msgs::msg::PointCloud2>(
    tobas::kLidarPointCloudTopic, kHesaiLatch, kHesaiReliable, kHesaiQueueSize);
  imu_raw_pub_ =
    createPublisher<sensor_msgs::msg::Imu>(tobas::kLidarImuTopic, kHesaiLatch, kHesaiReliable, kHesaiQueueSize);
}

void HesaiLidarPlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  // ベースフレームの状態を取得
  const auto& T_W_B = pose_W_->Data();
  const auto& R_W_B = T_W_B.Rot();
  const auto& acc_B = acc_B_->Data();
  const auto& gyro_B = gyro_B_->Data();
  const auto& dgyro_B = dgyro_B_->Data();

  // オフセットによる補正を考慮して加速度センサの読みを計算 (memo: 2-26)
  const auto grav_B = R_W_B.RotateVectorReverse(grav_W_->Data());
  const auto acc_corr = gyro_B.Cross(gyro_B.Cross(imu_params_.offset)) + dgyro_B.Cross(imu_params_.offset);
  auto acc_meas = acc_B - grav_B + acc_corr;

  // オフセットが並進のみならばジャイロセンサの読みはベースフレームの角速度に一致する
  auto gyro_meas = gyro_B;

  // Get delta time
  const auto dt = std::chrono::duration<double>(info.dt).count();

  // Add noise to the true values
  addImuNoise(acc_meas, gyro_meas, dt);

  // 姿勢推定の発散を防ぐために機体の位置姿勢が安定するまでは発行しない
  if (!stationary_state_detected_) {
    stationary_state_detected_ = (acc_B.Length() < kStaticAccThresh) && (gyro_B.Length() < kStaticGyroThresh);
    if (stationary_state_detected_) {
      TOBAS_INFO("Stationary state detected. Start to publish IMU messages.");
    }
    return;
  }

  // Publish rate filter
  if (!imu_rate_manager_->update(info.simTime)) {
    return;
  }

  // Get current time
  builtin_interfaces::msg::Time cur_time;
  ros2::timeChronoToMsg(info.simTime, cur_time);

  // Publish raw IMU message
  // Hesai ROS2 driverでの単位はaccはm/s^2, gyroはrad/s ref : 
  // https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0/blob/96be4a1fcbf74d41a04c74e12e5d5df694fab693/src/manager/source_driver_ros2.hpp#L374
  auto imu_raw_msg = std::make_unique<sensor_msgs::msg::Imu>();
  imu_raw_msg->header.stamp = cur_time;
  imu_raw_msg->header.frame_id = imu_params_.link_name;
  vectorGazeboToGeometryMsg(acc_meas, imu_raw_msg->linear_acceleration);
  vectorGazeboToGeometryMsg(gyro_meas, imu_raw_msg->angular_velocity);
  imu_raw_pub_->publish(std::move(imu_raw_msg));
}

void HesaiLidarPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  // load lidar parameters
  auto lidar_sdf = getSdfChild(sdf, "lidar");
  getLidarSdfParams(lidar_sdf);

  // load imu parameters
  auto imu_sdf = getSdfChild(sdf, "imu");
  getImuSdfParams(imu_sdf);
}

void HesaiLidarPlugin::getLidarSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "gpuRayTopic", lidar_params_.gpu_ray_topic);
  getSdfParam(sdf, "gpuRayUpdateRate", lidar_params_.gpu_ray_update_rate);
  getSdfParam(sdf, "updateRate", lidar_params_.update_rate);
  getSdfParam(sdf, "horizontalSamples", lidar_params_.horizontal_samples);
  getSdfParam(sdf, "verticalSamples", lidar_params_.vertical_samples);
  sampling_times_ = lidar_params_.gpu_ray_update_rate / lidar_params_.update_rate;
}

void HesaiLidarPlugin::getImuSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "linkName", imu_params_.link_name);
  getSdfParam(sdf, "updateRate", imu_params_.update_rate);
  getSdfParam(sdf, "offset", imu_params_.offset);
  getSdfParam(sdf, "gyroNoiseDensity", imu_params_.gyro_noise_density);
  getSdfParam(sdf, "gyroRandomWalk", imu_params_.gyro_random_walk);
  getSdfParam(sdf, "gyroBiasCorrelationTime", imu_params_.gyro_bias_corr_time);
  getSdfParam(sdf, "accelNoiseDensity", imu_params_.acc_noise_density);
  getSdfParam(sdf, "accelRandomWalk", imu_params_.acc_random_walk);
  getSdfParam(sdf, "accelBiasCorrelationTime", imu_params_.acc_bias_corr_time);
}

void HesaiLidarPlugin::gpuRayCb(const gz::msgs::PointCloudPacked& msg)
{
  // 初回か, publishした後でpoint_cloud_msgs_がnullptrになっている場合はデータを詰める
  if (phase_ == 0) {
    setupPointCloudMsg(msg);
  }

  bool is_final_phase = (phase_ == sampling_times_ - 1);

  // まんべんなくsamplingする
  uint32_t offset_time =
    convertToNanoSecond(msg.header().stamp().sec(), msg.header().stamp().nsec()) -
    convertToNanoSecond(point_cloud_msg_->header.stamp.sec, point_cloud_msg_->header.stamp.nanosec);
  double timestamp = offset_time * 1.0e-9;  // 単位は秒, Hesai Lidarのtime stampの単位と揃える
  for (int i = 0; i < lidar_params_.vertical_samples; i++) {
    // 各ringに対してsample点を見つける
    auto horizontal_samples = lidar_params_.horizontal_samples / sampling_times_;
    // もし割り切れない場合は端数がでるので一番最後のphaseで調整
    if (is_final_phase) {
      horizontal_samples = lidar_params_.horizontal_samples - horizontal_samples * (sampling_times_ - 1);
    }
    for (int j = 0; j < horizontal_samples; j++) {
      // point_cloud_msg_のデータのどのメモリ位置にデータを入れるか計算
      auto embedding_index = lidar_data_index_ * point_step_;
      // gpu_rayのデータのどの点をサンプリングするか，そのメモリの位置を計算
      auto sampling_idx = sampling_start_idx_[i] + j;
      if (sampling_idx >= msg.width()) {
        sampling_idx -= msg.width();
      }
      auto sampling_memory = i * msg.row_step() + sampling_idx * msg.point_step();
      // x, y, z
      memcpy(
        &point_cloud_msg_->data[embedding_index],
        &msg.data().c_str()[sampling_memory],
        sizeof(float) + sizeof(float) + sizeof(float));
      // intensity
      uint16_t intensity_i;
      memcpy(
        &intensity_i,
        &msg.data().c_str()[sampling_memory + sizeof(float) + sizeof(float) + sizeof(float)],
        sizeof(uint16_t));
      float intensity_f = intensity_i / 255.0;
      memcpy(&point_cloud_msg_->data[embedding_index + intensity_offset_], &intensity_f, sizeof(float));
      // ring
      auto ring = static_cast<uint16_t>(i);
      memcpy(&point_cloud_msg_->data[embedding_index + ring_offset_], &ring, sizeof(uint16_t));
      // timestamp
      memcpy(&point_cloud_msg_->data[embedding_index + timestamp_offset_], &timestamp, sizeof(double));

      // update
      lidar_data_index_++;
    }
    // 次のsamplingはずらした地点から行う
    sampling_start_idx_[i] += horizontal_samples;
    if (sampling_start_idx_[i] >= msg.width()) {
      sampling_start_idx_[i] -= msg.width();
    }
  }

  // update
  phase_++;

  // publish if sampling finished
  if (is_final_phase) {
    lidar_point_cloud_publisher_->publish(std::move(point_cloud_msg_));
    // reset index
    phase_ = 0;
    lidar_data_index_ = 0;
  }
}

void HesaiLidarPlugin::setupPointCloudMsg(const gz::msgs::PointCloudPacked& msg)
{
  point_cloud_msg_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
  // fill message field data to be consistent with the Hesai ROS2 driver
  // fill x
  sensor_msgs::msg::PointField field_x;
  field_x.name = "x";
  field_x.count = 1;
  field_x.offset = x_offset_;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_msg_->fields.push_back(field_x);
  // fill y
  sensor_msgs::msg::PointField field_y;
  field_y.name = "y";
  field_y.count = 1;
  field_y.offset = y_offset_;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_msg_->fields.push_back(field_y);
  // fill z
  sensor_msgs::msg::PointField field_z;
  field_z.name = "z";
  field_z.count = 1;
  field_z.offset = z_offset_;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_msg_->fields.push_back(field_z);
  // fill intensity
  sensor_msgs::msg::PointField field_intensity;
  field_intensity.name = "intensity";
  field_intensity.count = 1;
  field_intensity.offset = intensity_offset_;
  field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_msg_->fields.push_back(field_intensity);
  // fill ring
  sensor_msgs::msg::PointField field_ring;
  field_ring.name = "ring";
  field_ring.count = 1;
  field_ring.offset = ring_offset_;
  field_ring.datatype = sensor_msgs::msg::PointField::UINT16;
  point_cloud_msg_->fields.push_back(field_ring);
  // fill timestamp
  sensor_msgs::msg::PointField field_timestamp;
  field_timestamp.name = "timestamp";
  field_timestamp.count = 1;
  field_timestamp.offset = timestamp_offset_;
  field_timestamp.datatype = sensor_msgs::msg::PointField::FLOAT64;
  point_cloud_msg_->fields.push_back(field_timestamp);

  // setup other message data
  // Hesai ROS2 driverのstampはpublishした時刻ではなくてframe start timeであるらしい ref:
  // https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0/blob/96be4a1fcbf74d41a04c74e12e5d5df694fab693/src/manager/source_driver_ros2.hpp#L303
  point_cloud_msg_->header.frame_id = "map";
  point_cloud_msg_->header.stamp.sec = msg.header().stamp().sec();
  point_cloud_msg_->header.stamp.nanosec = msg.header().stamp().nsec();
  point_cloud_msg_->height = 1;
  point_cloud_msg_->width = lidar_params_.vertical_samples * lidar_params_.horizontal_samples;
  point_cloud_msg_->is_bigendian = msg.is_bigendian();
  point_cloud_msg_->point_step = point_step_;
  point_cloud_msg_->row_step = point_cloud_msg_->width * point_step_;
  point_cloud_msg_->is_dense = false;  // trying to be consistent with the Hesai ROS2 driver
  point_cloud_msg_->data.resize(lidar_params_.vertical_samples * lidar_params_.horizontal_samples * point_step_);
}

void HesaiLidarPlugin::addImuNoise(gz::math::Vector3d& acc, gz::math::Vector3d& gyro, const double& dt)
{
  // Accel
  const auto tau_a = imu_params_.acc_bias_corr_time;
  // Discrete-time std. dev equivalent to an "integrating" sampler with integration time dt
  const auto sigma_a_d = imu_params_.acc_noise_density / sqrt(dt);  // [m/s^2]
  const auto sigma_b_a = imu_params_.acc_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114] (memo: 2-32)
  const auto sigma_b_a_d = sigma_b_a * sqrt(tau_a / 2 * (1 - exp(-2 * dt / tau_a)));  // [m/s^2]
  // Compute state-transition
  const auto phi_a_d = exp(-dt / tau_a);
  // Simulate accelerometer noise processes and add them to the true linear acceleration
  acc_bias_ = phi_a_d * acc_bias_ + sigma_b_a_d * normal_.get();
  acc += sigma_a_d * normal_.get() + acc_bias_;

  // Gyro
  const auto tau_g = imu_params_.gyro_bias_corr_time;
  // Discrete-time std. dev equivalent to an "integrating" sampler with integration time dt
  const auto sigma_g_d = imu_params_.gyro_noise_density / sqrt(dt);  // [rad/s]
  const auto sigma_b_g = imu_params_.gyro_random_walk;
  // Compute exact covariance of the process after dt [Maybeck 4-114] (memo: 2-32)
  const auto sigma_b_g_d = sigma_b_g * sqrt(tau_g / 2 * (1 - exp(-2 * dt / tau_g)));  // [rad/s]
  // Compute state-transition
  const auto phi_g_d = exp(-dt / tau_g);
  // Simulate gyroscope noise processes and add them to the true angular rate
  gyro_bias_ = phi_g_d * gyro_bias_ + sigma_b_g_d * normal_.get();
  gyro += sigma_g_d * normal_.get() + gyro_bias_;
}

uint64_t HesaiLidarPlugin::convertToNanoSecond(const int64_t& sec, const int32_t& nanosec)
{
  return sec * static_cast<int>(1e9) + nanosec;
}

void HesaiLidarPlugin::vectorGazeboToGeometryMsg(const gz::math::Vector3d& gazebo_msg, geometry_msgs::msg::Vector3& msg)
{
  msg.x = gazebo_msg.X();
  msg.y = gazebo_msg.Y();
  msg.z = gazebo_msg.Z();
}

}  // namespace gazebo

GZ_ADD_PLUGIN(
  gazebo::HesaiLidarPlugin,
  gz::sim::System,
  gazebo::HesaiLidarPlugin::ISystemConfigure,
  gazebo::HesaiLidarPlugin::ISystemPostUpdate)
