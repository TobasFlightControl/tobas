#pragma once

#include <gz/rendering/GpuRays.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/RenderingSensor.hh>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tobas_constants/constants.hpp>

#include "tobas_gazebo_system_plugins/common/node.hpp"

namespace gazebo
{
class LivoxLidarSensor : public BaseNode,
                         public gz::sensors::RenderingSensor
{
public:
  explicit LivoxLidarSensor();

  void SetScene(gz::rendering::ScenePtr scene) override;
  void removeGpuRays(gz::rendering::ScenePtr scene);
  bool Load(const sdf::Sensor &sdf) override;
  bool Load(const sdf::ElementPtr sdf) override;
  bool Init() override;
  void SetParent(const std::string &_parent) override;
  bool HasConnections() const override;
  bool Update(const std::chrono::steady_clock::duration &_now) override;

private:
  static constexpr uint8_t kChannels = 3u; // lidar生データのchannel数. [range, intensity, REGISTERED]

  // SDF parameters
  uint samples_; // down samplingしない場合にlidarが取得する点群数
  uint downsample_; // 例えばdownsample_ = 3だと3つ飛ばしで点群データを読み取る
  uint downsampled_size_;
  sdf::Lidar sdf_lidar_; // settings of lidar sensor
  std::string csv_file_path_;
  gz::sensors::NoisePtr range_noise_model_;

  // lidar関連
  bool initialized_ = false;
  gz::rendering::GpuRaysPtr gpu_rays_;
  std::mutex lidar_mutex_;
  gz::common::ConnectionPtr lidar_frame_connection_; // Connection to gpu_rays_ new lidar frame event
  float* lidar_buffer_; // lidar生データのbuffer. livox lidarのrepetitive scan用に絞ることもdownsamplingもされていない.  [range, intensity, REGISTERED]
  sensor_msgs::msg::PointCloud2::UniquePtr lidar_msg_; // message to send

  // ros interfaces
  ros2::PublisherPtr<sensor_msgs::msg::PointCloud2> lidar_pub_;

  /* sdformat fileから設定情報を読み取る.  */
  void getSdfParams(const sdf::ElementConstPtr& sdf);
  bool createLidar();
  /* Lidarデータが来たときに呼ばれるcallback関数. データをlidar_buffer_へ保存する. */
  void onNewLidarFrame(const float *scan,
    unsigned int width, unsigned int height, unsigned int channels,
    const std::string &format);
  /* Lidar scanのデータをros messageで発行する. */
  void publishLidarScan();
};
} // end of namepsace gazebo
