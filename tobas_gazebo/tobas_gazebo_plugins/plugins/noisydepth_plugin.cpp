#include <gazebo/rendering/DepthCamera.hh>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "./noisydepth_plugin.hpp"
#include "../include/tobas_gazebo_plugins/sdfparam.hpp"
#include "../include/tobas_gazebo_plugins/conversions/gazebo_ros.hpp"

using namespace std;

namespace gazebo
{
GazeboNoisyDepthPlugin::GazeboNoisyDepthPlugin()
{
}

GazeboNoisyDepthPlugin::~GazeboNoisyDepthPlugin()
{
  parent_sensor_.reset();
  depth_camera_.reset();
}

void GazeboNoisyDepthPlugin::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
  gzmsg << "Loading " << kPluginName << "." << endl;

  parent_sensor_ = dynamic_pointer_cast<sensors::DepthCameraSensor>(parent);
  if (!parent_sensor_)
    gzthrow(kPluginName << ": Depth camera sensor is not attached.");
  depth_camera_ = parent_sensor_->DepthCamera();

  // Copy from DepthCamera into GazeboRosCameraUtils
  parentSensor_ = parent_sensor_;
  camera_ = depth_camera_;
  width_ = depth_camera_->ImageWidth();
  height_ = depth_camera_->ImageHeight();
  depth_ = depth_camera_->ImageDepth();
  format_ = depth_camera_->ImageFormat();

  getSdfParams(sdf);
  setNoiseModel();

  // Listen to the update events
  new_image_frame_connection_ =
    depth_camera_->ConnectNewImageFrame(std::bind(&self::onNewImageFrame, this, _1, _2, _3, _4, _5));
  new_depth_frame_connection_ =
    depth_camera_->ConnectNewDepthFrame(std::bind(&self::onNewDepthFrame, this, _1, _2, _3, _4, _5));

  // GazeboRosCameraUtilsのLoadが完了してからadvertiseを行うように設定する
  // これをせずadvertiseをベタ書きするとsegmentation faultになる
  load_connection_ = GazeboRosCameraUtils::OnLoad(std::bind(&self::advertise, this));
  GazeboRosCameraUtils::Load(parent, sdf);

  parent_sensor_->SetActive(true);
}

void GazeboNoisyDepthPlugin::onNewImageFrame(
  const uint8_t* image,
  size_t width,
  size_t height,
  size_t depth,
  const string& format)
{
  if (!initialized_ || height <= 0 || width <= 0)
    return;

  sensor_update_time_ = parent_sensor_->LastMeasurementTime();

  // Check if there are subscribers, if not disable parent, else process images..
  if (parent_sensor_->IsActive())
  {
    if (depth_image_connect_count_ <= 0 && (*image_connect_count_) <= 0)
      parent_sensor_->SetActive(false);
    else if ((*image_connect_count_) > 0)
      PutCameraData(image);
  }
  else
  {
    // If parent is disabled, but has subscribers, enable it.
    if ((*image_connect_count_) > 0)
      parent_sensor_->SetActive(true);
  }
}

void GazeboNoisyDepthPlugin::onNewDepthFrame(
  const float* image,
  size_t width,
  size_t height,
  size_t depth,
  const string& format)
{
  if (!initialized_ || height <= 0 || width <= 0)
    return;

  // Check if there are subscribers, if not disable parent, else process images.
  if (parent_sensor_->IsActive())
  {
    if (depth_image_connect_count_ <= 0 && (*image_connect_count_) <= 0)
      parent_sensor_->SetActive(false);
    else if (depth_image_connect_count_ > 0)
      publishDepthImage(image);
  }
  else
  {
    // If parent is disabled, but has subscribers, enable it.
    if ((*image_connect_count_) > 0)
      parent_sensor_->SetActive(true);
  }

  publishCameraInfo();
}

void GazeboNoisyDepthPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "irImageTopic", image_topic_name_, kDefaultIrImageTopic);
  getSdfParam(sdf, "irInfoTopic", camera_info_topic_name_, kDefaultIrInfoTopic);
  getSdfParam(sdf, "depthImageTopic", depth_image_topic_, kDefaultDepthImageTopic);
  getSdfParam(sdf, "depthInfoTopic", depth_info_topic_, kDefaultDepthInfoTopic);

  getSdfParam(sdf, "depthNoiseModel", noise_model_name_, kDefaultDepthNoiseModel);

  getSdfParam(sdf, "depthNoiseMinDist", noise_min_dist_, kDefaultDepthNoiseMinDist, NON_NEGATIVE);
  getSdfParam(sdf, "depthNoiseMaxDist", noise_max_dist_, kDefaultDepthNoiseMaxDist, NON_NEGATIVE);
  if (noise_min_dist_ >= noise_max_dist_)
    gzthrow(kPluginName << ": Invalid noise distance range.");

  getSdfParam(sdf, "horizontalFOV", horizontal_fov_, kDefaultHorizontalFOV, POSITIVE);
  getSdfParam(sdf, "baseline", baseline_, kDefaultBaseline, POSITIVE);
}

void GazeboNoisyDepthPlugin::setNoiseModel()
{
  if (noise_model_name_ == "Kinect")
    noise_model_.reset(new KinectDepthNoiseModel(noise_min_dist_, noise_max_dist_));
  else if (noise_model_name_ == "PMD")
    noise_model_.reset(new PMDDepthNoiseModel(noise_min_dist_, noise_max_dist_));
  else if (noise_model_name_ == "D435")
    noise_model_.reset(new D435DepthNoiseModel(noise_min_dist_, noise_max_dist_, horizontal_fov_, baseline_));
  else
    gzthrow(kPluginName << ": Invalid depth noise model: " << noise_model_name_);
}

void GazeboNoisyDepthPlugin::advertise()
{
  rclcpp::AdvertiseOptions depth_image_ao = rclcpp::AdvertiseOptions::create<sensor_msgs::msg::Image>(
    depth_image_topic_, 1, std::bind(&GazeboNoisyDepthPlugin::depthImageConnect, this),
    std::bind(&GazeboNoisyDepthPlugin::depthImageDisconnect, this), rclcpp::VoidPtr(), &camera_queue_);

  depth_image_pub_ = rosnode_->advertise(depth_image_ao);

  rclcpp::AdvertiseOptions depth_info_ao = rclcpp::AdvertiseOptions::create<sensor_msgs::msg::CameraInfo>(
    depth_info_topic_, 1, std::bind(&GazeboNoisyDepthPlugin::depthInfoConnect, this),
    std::bind(&GazeboNoisyDepthPlugin::depthInfoDisconnect, this), rclcpp::VoidPtr(), &camera_queue_);

  depth_info_pub_ = rosnode_->advertise(depth_info_ao);
}

void GazeboNoisyDepthPlugin::depthImageConnect()
{
  ++depth_image_connect_count_;
  parent_sensor_->SetActive(true);
}

void GazeboNoisyDepthPlugin::depthImageDisconnect()
{
  --depth_image_connect_count_;
}

void GazeboNoisyDepthPlugin::depthInfoConnect()
{
  ++depth_info_connect_count_;
}

void GazeboNoisyDepthPlugin::depthInfoDisconnect()
{
  --depth_info_connect_count_;
}

void GazeboNoisyDepthPlugin::publishDepthImage(const float* src)
{
  if (src == nullptr)
  {
    gzwarn << "[" << kPluginName << "] Image source is null." << endl;
    return;
  }

  lock_.lock();  // TODO: ロックはなぜ必要？

  // Create an image message
  const auto image_msg =std::make_unique<sensor_msgs::msg::Image>();

  // Fill header
  timeGazeboToRos(parent_sensor_->LastMeasurementTime(), image_msg->header.stamp);
  image_msg->header.frame_id = frame_name_;

  // Fill basic information
  image_msg->height = height_;
  image_msg->width = width_;
  image_msg->encoding = sensor_msgs::msg::image_encodings::TYPE_32FC1;
  image_msg->is_bigendian = 0;
  image_msg->step = sizeof(float) * width_;

  // Fill image data
  image_msg->data.resize(height_ * width_ * sizeof(float));
  float* dest = (float*)(&(image_msg->data[0]));
  memcpy(dest, src, sizeof(float) * width_ * height_);

  // Add noise to image data
  noise_model_->applyNoise(width_, height_, dest);

  // Publish image
  depth_image_pub_->publish(image_msg);

  lock_.unlock();
}

void GazeboNoisyDepthPlugin::publishCameraInfo()
{
  // First publish parent camera info (IR camera)
  PublishCameraInfo();

  if (depth_info_connect_count_ <= 0)
    return;

  sensor_update_time_ = parentSensor_->LastMeasurementTime();
  if (sensor_update_time_ - last_depth_info_update_time_ >= update_period_)
  {
    PublishCameraInfo(depth_info_pub_);
    last_depth_info_update_time_ = sensor_update_time_;
  }
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboNoisyDepthPlugin)
}  // namespace gazebo
