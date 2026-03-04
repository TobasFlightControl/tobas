#include <bit>

#include <gz/transport/Node.hh>

#include <tobas_gazebo_system_plugins/common/common.hpp>
#include <tobas_gazebo_system_plugins/random.hpp>
#include <tobas_gazebo_system_plugins/rate_manager.hpp>
#include <tobas_gazebo_tools/time.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace cmp = gz::sim::components;

namespace gazebo
{
/**
 * @brief Gazebo の GPU LiDAR を ROS 2 で Hesai LiDAR っぽく使用できるようにブリッジする．
 *
 * gpu_lidar センサと gz-sim-sensors-system プラグインにより発出される
 * gz::msgs::PointCloudPacked 型のメッセージを購読し，
 * ROS 2 の sensor_msgs::msg::PointCloud2 型のメッセージとして再発行する．
 */
class HesaiLidarPlugin : public BaseNode, public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  explicit HesaiLidarPlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager& event_mgr) override;

private:
  static constexpr char kHesaiLidarTopic[] = "hesai_lidar";
  static constexpr bool kHesaiLatch = true;
  static constexpr bool kHesaiReliable = true;
  static constexpr int kHesaiQueueSize = 10;

  struct Params
  {
    std::string gpu_ray_topic;
    int gpu_ray_update_rate;
    int update_rate;
    int horizontal_samples;
    int vertical_samples;
  } lidar_params_;

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
  std::vector<uint32_t> sampling_start_idx_;
  // lidarのデータのどのindexのところにデータをつめるか
  uint32_t lidar_data_index_ = 0;

  // gazebo interfaces
  gz::transport::Node node_;

  // ros2 interfaces
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud2> point_cloud_publisher_;
  sensor_msgs::msg::PointCloud2::UniquePtr point_cloud_msg_;

  // load configurations from sdf
  void getSdfParams(const sdf::ElementConstPtr& sdf);

  // lidar related functions
  void gpuRayCb(const gz::msgs::PointCloudPacked& msg);
  void setupPointCloudMsg(const gz::msgs::PointCloudPacked& msg);
};

HesaiLidarPlugin::HesaiLidarPlugin()
{
}

void HesaiLidarPlugin::Configure(
  const gz::sim::Entity&,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager&,
  gz::sim::EventManager&)
{
  initialize("hesai_lidar_plugin", sdf);
  getSdfParams(sdf);

  // setup useful parameters
  x_offset_ = 0;
  y_offset_ = x_offset_ + sizeof(float);
  z_offset_ = y_offset_ + sizeof(float);
  intensity_offset_ = z_offset_ + sizeof(float);
  ring_offset_ = intensity_offset_ + sizeof(float);
  timestamp_offset_ = ring_offset_ + sizeof(uint16_t);
  point_step_ = timestamp_offset_ + sizeof(double);
  sampling_start_idx_.resize(lidar_params_.vertical_samples);
  for (size_t i = 0; i < sampling_start_idx_.size(); ++i) {
    sampling_start_idx_[i] = std::floor(
      i * static_cast<float>(lidar_params_.horizontal_samples) / static_cast<float>(lidar_params_.vertical_samples));
  }

  // gazebo interfaces
  node_.Subscribe(lidar_params_.gpu_ray_topic, &HesaiLidarPlugin::gpuRayCb, this);

  // ros interfaces
  point_cloud_publisher_ =
    createPublisher<sensor_msgs::msg::PointCloud2>(kHesaiLidarTopic, kHesaiLatch, kHesaiReliable, kHesaiQueueSize);
}

void HesaiLidarPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "gpuRayTopic", lidar_params_.gpu_ray_topic);
  getSdfParam(sdf, "gpuRayUpdateRate", lidar_params_.gpu_ray_update_rate);
  getSdfParam(sdf, "updateRate", lidar_params_.update_rate);
  getSdfParam(sdf, "horizontalSamples", lidar_params_.horizontal_samples);
  getSdfParam(sdf, "verticalSamples", lidar_params_.vertical_samples);
  sampling_times_ = lidar_params_.gpu_ray_update_rate / lidar_params_.update_rate;
}

void HesaiLidarPlugin::gpuRayCb(const gz::msgs::PointCloudPacked& msg)
{
  // 初回か, publishした後でpoint_cloud_msgs_がnullptrになっている場合はデータを詰める
  if (phase_ == 0) {
    setupPointCloudMsg(msg);
  }

  const auto is_final_phase = (phase_ == sampling_times_ - 1);

  // まんべんなくsamplingする
  const auto offset_time = nanoseconds(msg.header().stamp()) - ros2::nanoseconds(point_cloud_msg_->header.stamp);
  const double timestamp = static_cast<double>(offset_time) * 1e-9;  // 単位は秒, Hesai Lidarのtime stampの単位と揃える
  for (int i = 0; i < lidar_params_.vertical_samples; ++i) {
    // 各ringに対してsample点を見つける
    auto horizontal_samples = lidar_params_.horizontal_samples / sampling_times_;
    // もし割り切れない場合は端数がでるので一番最後のphaseで調整
    if (is_final_phase) {
      horizontal_samples = lidar_params_.horizontal_samples - horizontal_samples * (sampling_times_ - 1);
    }
    for (int j = 0; j < horizontal_samples; ++j) {
      // point_cloud_msg_のデータのどのメモリ位置にデータを入れるか計算
      const auto embedding_index = lidar_data_index_ * point_step_;
      // gpu_rayのデータのどの点をサンプリングするか，そのメモリの位置を計算
      auto sampling_idx = sampling_start_idx_[i] + j;
      if (sampling_idx >= msg.width()) {
        sampling_idx -= msg.width();
      }
      const auto sampling_memory = i * msg.row_step() + sampling_idx * msg.point_step();
      // x, y, z
      memcpy(&point_cloud_msg_->data[embedding_index], &msg.data().c_str()[sampling_memory], sizeof(float) * 3);
      // intensity
      uint16_t intensity_i;
      memcpy(&intensity_i, &msg.data().c_str()[sampling_memory + sizeof(float) * 3], sizeof(uint16_t));
      const float intensity_f = static_cast<float>(intensity_i) / 255.0f;
      memcpy(&point_cloud_msg_->data[embedding_index + intensity_offset_], &intensity_f, sizeof(float));
      // ring
      const uint16_t ring = static_cast<uint16_t>(i);
      memcpy(&point_cloud_msg_->data[embedding_index + ring_offset_], &ring, sizeof(uint16_t));
      // timestamp
      memcpy(&point_cloud_msg_->data[embedding_index + timestamp_offset_], &timestamp, sizeof(double));

      // update
      ++lidar_data_index_;
    }
    // 次のsamplingはずらした地点から行う
    sampling_start_idx_[i] += horizontal_samples;
    if (sampling_start_idx_[i] >= msg.width()) {
      sampling_start_idx_[i] -= msg.width();
    }
  }

  // update
  ++phase_;

  // publish if sampling finished
  if (is_final_phase) {
    point_cloud_publisher_->publish(std::move(point_cloud_msg_));
    // reset index
    phase_ = 0;
    lidar_data_index_ = 0;
  }
}

void HesaiLidarPlugin::setupPointCloudMsg(const gz::msgs::PointCloudPacked& msg)
{
  point_cloud_msg_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
  // fill message field data to be consistent with the Hesai ROS 2 driver
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
  // Hesai ROS 2 driverのstampはpublishした時刻ではなくてframe start timeであるらしい ref:
  // https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0/blob/96be4a1fcbf74d41a04c74e12e5d5df694fab693/src/manager/source_driver_ros2.hpp#L303
  point_cloud_msg_->header.frame_id = "map";
  point_cloud_msg_->header.stamp.sec = msg.header().stamp().sec();
  point_cloud_msg_->header.stamp.nanosec = msg.header().stamp().nsec();
  point_cloud_msg_->height = 1;
  point_cloud_msg_->width = lidar_params_.vertical_samples * lidar_params_.horizontal_samples;
  point_cloud_msg_->is_bigendian = msg.is_bigendian();
  point_cloud_msg_->point_step = point_step_;
  point_cloud_msg_->row_step = point_cloud_msg_->width * point_step_;
  point_cloud_msg_->is_dense = false;  // trying to be consistent with the Hesai ROS 2 driver
  point_cloud_msg_->data.resize(lidar_params_.vertical_samples * lidar_params_.horizontal_samples * point_step_);
}
}  // namespace gazebo

GZ_ADD_PLUGIN(gazebo::HesaiLidarPlugin, gz::sim::System, gazebo::HesaiLidarPlugin::ISystemConfigure)
