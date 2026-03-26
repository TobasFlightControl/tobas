#include <gz/transport/Node.hh>

#include <tobas_gazebo_tools/time.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
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
  struct SdfParams
  {
    std::string topic;
    int update_rate;
    std::string parent_topic;
    int parent_update_rate;
    int horizontal_samples;
    int vertical_samples;
  } params_;

  // LiDAR関連
  // 1回発行するまでに何回gpu_rayをサンプリングするか
  int sampling_times_;
  // gpu_lidarを前の発行から数えて何回受信して詰めたか
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
  // 各ringに対して, 水平方向のサンプリングを始めるインデックス
  // 点群データを歪ませるためにgpu_rayのデータからこのインデックスから開始した点のみを間引いてサンプリングする
  std::vector<uint32_t> sampling_start_idx_;
  // LiDARのデータのどのインデックスのところにデータをつめるか
  uint32_t lidar_data_index_ = 0;

  // Gazebo interfaces
  gz::transport::Node node_;

  // ROS interfaces
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud2> point_cloud_publisher_;
  sensor_msgs::msg::PointCloud2::UniquePtr point_cloud_msg_;

  // Load configurations from SDF
  void getSdfParams(const sdf::ElementConstPtr& sdf);

  // LiDAR related functions
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

  sampling_times_ = params_.parent_update_rate / params_.update_rate;
  x_offset_ = 0;
  y_offset_ = x_offset_ + sizeof(float);
  z_offset_ = y_offset_ + sizeof(float);
  intensity_offset_ = z_offset_ + sizeof(float);
  ring_offset_ = intensity_offset_ + sizeof(float);
  timestamp_offset_ = ring_offset_ + sizeof(uint16_t);
  point_step_ = timestamp_offset_ + sizeof(double);
  sampling_start_idx_.resize(params_.vertical_samples);
  for (size_t i = 0; i < sampling_start_idx_.size(); ++i) {
    sampling_start_idx_[i] =
      std::floor(i * static_cast<float>(params_.horizontal_samples) / static_cast<float>(params_.vertical_samples));
  }

  // Gazebo interfaces
  node_.Subscribe(params_.parent_topic, &HesaiLidarPlugin::gpuRayCb, this);

  // ROS interfaces
  point_cloud_publisher_ = createPublisher<sensor_msgs::msg::PointCloud2>(params_.topic, true, true, 10);
}

void HesaiLidarPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "topic", params_.topic);
  getSdfParam(sdf, "updateRate", params_.update_rate);
  getSdfParam(sdf, "parentSensorTopic", params_.parent_topic);
  getSdfParam(sdf, "parentSensorUpdateRate", params_.parent_update_rate);
  getSdfParam(sdf, "parentSensorHorizontalSamples", params_.horizontal_samples);
  getSdfParam(sdf, "parentSensorVerticalSamples", params_.vertical_samples);
}

void HesaiLidarPlugin::gpuRayCb(const gz::msgs::PointCloudPacked& msg)
{
  // 初回か発行した後でpoint_cloud_msg_がnullptrになっている場合はデータを詰める
  if (phase_ == 0) {
    setupPointCloudMsg(msg);
  }

  const auto is_final_phase = (phase_ == sampling_times_ - 1);

  // まんべんなくサンプリングする
  const auto offset_time = nanoseconds(msg.header().stamp()) - ros2::nanoseconds(point_cloud_msg_->header.stamp);
  const double timestamp = static_cast<double>(offset_time) * 1e-9;  // 単位は秒, Hesai Lidarのtime stampの単位と揃える
  for (int i = 0; i < params_.vertical_samples; ++i) {
    // 各ringに対してsample点を見つける
    auto horizontal_samples = params_.horizontal_samples / sampling_times_;
    // もし割り切れない場合は端数がでるので一番最後のphaseで調整
    if (is_final_phase) {
      horizontal_samples = params_.horizontal_samples - horizontal_samples * (sampling_times_ - 1);
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
      // Intensity
      uint16_t intensity_i;
      memcpy(&intensity_i, &msg.data().c_str()[sampling_memory + sizeof(float) * 3], sizeof(uint16_t));
      const float intensity_f = static_cast<float>(intensity_i) / 255.0f;
      memcpy(&point_cloud_msg_->data[embedding_index + intensity_offset_], &intensity_f, sizeof(float));
      // Ring
      const uint16_t ring = static_cast<uint16_t>(i);
      memcpy(&point_cloud_msg_->data[embedding_index + ring_offset_], &ring, sizeof(uint16_t));
      // Timestamp
      memcpy(&point_cloud_msg_->data[embedding_index + timestamp_offset_], &timestamp, sizeof(double));

      // Update
      ++lidar_data_index_;
    }
    // 次のsamplingはずらした地点から行う
    sampling_start_idx_[i] += horizontal_samples;
    if (sampling_start_idx_[i] >= msg.width()) {
      sampling_start_idx_[i] -= msg.width();
    }
  }

  // Update
  ++phase_;

  // Publish if sampling finished
  if (is_final_phase) {
    point_cloud_publisher_->publish(std::move(point_cloud_msg_));
    // Reset index
    phase_ = 0;
    lidar_data_index_ = 0;
  }
}

void HesaiLidarPlugin::setupPointCloudMsg(const gz::msgs::PointCloudPacked& msg)
{
  point_cloud_msg_ = std::make_unique<sensor_msgs::msg::PointCloud2>();
  // Fill message field data to be consistent with the Hesai ROS 2 driver
  // Fill x
  sensor_msgs::msg::PointField field_x;
  field_x.name = "x";
  field_x.count = 1;
  field_x.offset = x_offset_;
  field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_msg_->fields.push_back(field_x);
  // Fill y
  sensor_msgs::msg::PointField field_y;
  field_y.name = "y";
  field_y.count = 1;
  field_y.offset = y_offset_;
  field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_msg_->fields.push_back(field_y);
  // Fill z
  sensor_msgs::msg::PointField field_z;
  field_z.name = "z";
  field_z.count = 1;
  field_z.offset = z_offset_;
  field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_msg_->fields.push_back(field_z);
  // Fill intensity
  sensor_msgs::msg::PointField field_intensity;
  field_intensity.name = "intensity";
  field_intensity.count = 1;
  field_intensity.offset = intensity_offset_;
  field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
  point_cloud_msg_->fields.push_back(field_intensity);
  // Fill ring
  sensor_msgs::msg::PointField field_ring;
  field_ring.name = "ring";
  field_ring.count = 1;
  field_ring.offset = ring_offset_;
  field_ring.datatype = sensor_msgs::msg::PointField::UINT16;
  point_cloud_msg_->fields.push_back(field_ring);
  // Fill timestamp
  sensor_msgs::msg::PointField field_timestamp;
  field_timestamp.name = "timestamp";
  field_timestamp.count = 1;
  field_timestamp.offset = timestamp_offset_;
  field_timestamp.datatype = sensor_msgs::msg::PointField::FLOAT64;
  point_cloud_msg_->fields.push_back(field_timestamp);

  // Setup other message data
  // Hesai ROS 2 driverのstampはpublishした時刻ではなくてframe start timeであるらしい
  // https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0/blob/96be4a1fcbf74d41a04c74e12e5d5df694fab693/src/manager/source_driver_ros2.hpp#L303
  point_cloud_msg_->header.frame_id = "map";
  point_cloud_msg_->header.stamp.sec = msg.header().stamp().sec();
  point_cloud_msg_->header.stamp.nanosec = msg.header().stamp().nsec();
  point_cloud_msg_->height = 1;
  point_cloud_msg_->width = params_.vertical_samples * params_.horizontal_samples;
  point_cloud_msg_->is_bigendian = msg.is_bigendian();
  point_cloud_msg_->point_step = point_step_;
  point_cloud_msg_->row_step = point_cloud_msg_->width * point_step_;
  point_cloud_msg_->is_dense = false;  // Trying to be consistent with the Hesai ROS 2 driver
  point_cloud_msg_->data.resize(params_.vertical_samples * params_.horizontal_samples * point_step_);
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(tobas::gazebo::HesaiLidarPlugin, gz::sim::System, tobas::gazebo::HesaiLidarPlugin::ISystemConfigure)
