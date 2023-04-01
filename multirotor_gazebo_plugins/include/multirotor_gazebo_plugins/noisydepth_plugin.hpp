#pragma once

#include <ros/ros.h>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <sensor_msgs/Image.h>

#include "./depth_noise_models.hpp"

namespace gazebo
{
// Constants
static constexpr char kPluginName[] = "noisydepth_plugin";

// Default values
static constexpr char kDefaultIrImageTopic[] = "ir/image_raw";
static constexpr char kDefaultIrInfoTopic[] = "ir/image_info";
static constexpr char kDefaultDepthImageTopic[] = "depth/image_raw";
static constexpr char kDefaultDepthInfoTopic[] = "depth/image_info";
static constexpr char kDefaultDepthNoiseModel[] = "Kinect";
static constexpr float kDefaultDepthNoiseMinDist = 0.0f;
static constexpr float kDefaultDepthNoiseMaxDist = 1e+9f;

class GazeboNoisyDepth : public DepthCameraPlugin, GazeboRosCameraUtils
{
public:
  GazeboNoisyDepth();

  void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

  void OnNewImageFrame(
    const u_char* image,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    const std::string& format) override;

  void OnNewDepthFrame(
    const float* image,
    uint32_t width,
    uint32_t height,
    uint32_t depth,
    const std::string& format) override;

private:
  // SDF parameters
  std::string depth_image_topic_;
  std::string depth_info_topic_;
  std::string noise_model_name_;
  float noise_min_dist_;
  float noise_max_dist_;

  std::unique_ptr<DepthNoiseModel> noise_model_;
  int depth_image_connect_count_;
  int depth_info_connect_count_;
  common::Time depth_sensor_update_time_;
  common::Time last_depth_info_update_time_;
  sensor_msgs::Image depth_image_msg_;

  ros::Publisher depth_image_pub_;
  ros::Publisher depth_info_pub_;

  event::ConnectionPtr update_connection_;

  void getSdfParams(sdf::ElementPtr sdf);
  void setNoiseModel();
  void advertise();
  void depthImageConnect();
  void depthImageDisconnect();
  void depthInfoConnect();
  void depthInfoDisconnect();
  void fillDepthImage(const float* src);
  bool fillDepthImageHelper(
    const uint32_t rows_arg,
    const uint32_t cols_arg,
    const uint32_t step_arg,
    const float* data_arg,
    sensor_msgs::Image& image_msg);
  void publishCameraInfo();
};
}  // namespace gazebo
