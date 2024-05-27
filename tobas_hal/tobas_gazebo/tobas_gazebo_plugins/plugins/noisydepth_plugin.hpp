#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
#include <sensor_msgs/Image.h>

#include "../include/tobas_gazebo_plugins/depth_noise_models.hpp"

namespace gazebo
{
/**
 * @brief DepthCamera + Noise
 * cf. https://github.com/gazebosim/gazebo-classic/blob/gazebo11/plugins/DepthCameraPlugin.cc
 */
class GazeboNoisyDepthPlugin : public SensorPlugin, GazeboRosCameraUtils
{
  // Constants
  static constexpr char kPluginName[] = "noisydepth_plugin";

  // Default values
  const std::string kDefaultIrImageTopic = "ir/image_raw";
  const std::string kDefaultIrInfoTopic = "ir/image_info";
  const std::string kDefaultDepthImageTopic = "depth/image_raw";
  const std::string kDefaultDepthInfoTopic = "depth/image_info";
  const std::string kDefaultDepthNoiseModel = "Kinect";
  static constexpr float kDefaultDepthNoiseMinDist = 0.0f;
  static constexpr float kDefaultDepthNoiseMaxDist = 1e+9f;
  static constexpr float kDefaultHorizontalFOV = M_PI_2f32;
  static constexpr float kDefaultBaseline = 0.05f;

public:
  explicit GazeboNoisyDepthPlugin();
  ~GazeboNoisyDepthPlugin();

  void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override;

private:
  sensors::DepthCameraSensorPtr parent_sensor_;
  rendering::DepthCameraPtr depth_camera_;

  // SDF parameters
  std::string depth_image_topic_;
  std::string depth_info_topic_;
  std::string noise_model_name_;
  float noise_min_dist_;
  float noise_max_dist_;
  float horizontal_fov_;
  float baseline_;

  std::unique_ptr<DepthNoiseModel> noise_model_;
  int depth_image_connect_count_;
  int depth_info_connect_count_;
  common::Time depth_sensor_update_time_;
  common::Time last_depth_info_update_time_;
  sensor_msgs::Image depth_image_msg_;

  event::ConnectionPtr new_image_frame_connection_;
  event::ConnectionPtr new_depth_frame_connection_;
  event::ConnectionPtr load_connection_;

  ros::Publisher depth_image_pub_;
  ros::Publisher depth_info_pub_;

  void onNewImageFrame(const uint8_t* image, size_t width, size_t height, size_t depth, const std::string& format);

  void onNewDepthFrame(const float* image, size_t width, size_t height, size_t depth, const std::string& format);

  void getSdfParams(sdf::ElementPtr sdf);
  void setNoiseModel();
  void advertise();
  void depthImageConnect();
  void depthImageDisconnect();
  void depthInfoConnect();
  void depthInfoDisconnect();
  void fillDepthImage(const float* src);
  bool fillDepthImageHelper(
    const size_t rows_arg,
    const size_t cols_arg,
    const size_t step_arg,
    const float* data_arg,
    sensor_msgs::Image& image_msg);
  void publishCameraInfo();
};
}  // namespace gazebo
