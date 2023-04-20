#include <gazebo/rendering/DepthCamera.hh>
#include <sensor_msgs/image_encodings.h>

#include "../../include/plugins/noisydepth_plugin.hpp"
#include "../../include/tobas_gazebo_plugins/utils.hpp"
#include "../../include/tobas_gazebo_plugins/conversions.hpp"

using namespace std;

namespace gazebo
{
GazeboNoisyDepthPlugin::GazeboNoisyDepthPlugin()
{
  depth_info_connect_count_ = 0;
  depth_image_connect_count_ = 0;
  last_depth_info_update_time_ = common::Time(0);
}

GazeboNoisyDepthPlugin::~GazeboNoisyDepthPlugin()
{
  parent_sensor_.reset();
  depth_camera_.reset();
}

void GazeboNoisyDepthPlugin::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
  parent_sensor_ = dynamic_pointer_cast<sensors::DepthCameraSensor>(parent);
  if (!parent_sensor_)
  {
    gzthrow(kPluginName << ": Depth camera sensor is not attached.");
  }
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
  new_image_frame_connection_ = depth_camera_->ConnectNewImageFrame(
    boost::bind(&GazeboNoisyDepthPlugin::onNewImageFrame, this, _1, _2, _3, _4, _5));
  new_depth_frame_connection_ = depth_camera_->ConnectNewDepthFrame(
    boost::bind(&GazeboNoisyDepthPlugin::onNewDepthFrame, this, _1, _2, _3, _4, _5));

  // GazeboRosCameraUtilsのLoadが完了してからadvertiseを行うように設定する
  // これをせずadvertiseをベタ書きするとsegmentation faultになる
  load_connection_ =
    GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboNoisyDepthPlugin::advertise, this));
  GazeboRosCameraUtils::Load(parent, sdf);

  parent_sensor_->SetActive(true);
}

void GazeboNoisyDepthPlugin::onNewImageFrame(
  const u_char* image,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  const string& format)
{
  if (!initialized_ || height <= 0 || width <= 0)
  {
    return;
  }

  sensor_update_time_ = parent_sensor_->LastMeasurementTime();

  // Check if there are subscribers, if not disable parent, else process images..
  if (parent_sensor_->IsActive())
  {
    if (depth_image_connect_count_ <= 0 && (*image_connect_count_) <= 0)
    {
      parent_sensor_->SetActive(false);
    }
    else
    {
      if ((*image_connect_count_) > 0)
      {
        PutCameraData(image);
      }
    }
  }
  else
  {
    if ((*image_connect_count_) > 0)
    {
      // If parent is disabled, but has subscribers, enable it.
      parent_sensor_->SetActive(true);
    }
  }
}

void GazeboNoisyDepthPlugin::onNewDepthFrame(
  const float* image,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  const string& format)
{
  if (!initialized_ || height <= 0 || width <= 0)
  {
    return;
  }

  depth_sensor_update_time_ = parent_sensor_->LastMeasurementTime();

  // Check if there are subscribers, if not disable parent, else process images.
  if (parent_sensor_->IsActive())
  {
    if (depth_image_connect_count_ <= 0 && (*image_connect_count_) <= 0)
    {
      parent_sensor_->SetActive(false);
    }
    else
    {
      if (depth_image_connect_count_ > 0)
      {
        fillDepthImage(image);
      }
    }
  }
  else
  {
    // If parent is disabled, but has subscribers, enable it.
    if ((*image_connect_count_) > 0)
    {
      parent_sensor_->SetActive(true);
    }
  }

  publishCameraInfo();
}

void GazeboNoisyDepthPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  getSdfParam<string>(sdf, "irImageTopic", image_topic_name_, kDefaultIrImageTopic);
  getSdfParam<string>(sdf, "irInfoTopic", camera_info_topic_name_, kDefaultIrInfoTopic);
  getSdfParam<string>(sdf, "depthImageTopic", depth_image_topic_, kDefaultDepthImageTopic);
  getSdfParam<string>(sdf, "depthInfoTopic", depth_info_topic_, kDefaultDepthInfoTopic);
  getSdfParam<string>(sdf, "depthNoiseModel", noise_model_name_, kDefaultDepthNoiseModel);
  getSdfParam<float>(sdf, "depthNoiseMinDist", noise_min_dist_, kDefaultDepthNoiseMinDist);
  getSdfParam<float>(sdf, "depthNoiseMaxDist", noise_max_dist_, kDefaultDepthNoiseMaxDist);
  getSdfParam<float>(sdf, "horizontalFOV", horizontal_fov_, kDefaultHorizontalFOV);
  getSdfParam<float>(sdf, "baseline", baseline_, kDefaultBaseline);
}

void GazeboNoisyDepthPlugin::setNoiseModel()
{
  if (noise_model_name_ == "Kinect")
  {
    noise_model_.reset(new KinectDepthNoiseModel(noise_min_dist_, noise_max_dist_));
  }
  else if (noise_model_name_ == "PMD")
  {
    noise_model_.reset(new PMDDepthNoiseModel(noise_min_dist_, noise_max_dist_));
  }
  else if (noise_model_name_ == "D435")
  {
    noise_model_.reset(
      new D435DepthNoiseModel(noise_min_dist_, noise_max_dist_, horizontal_fov_, baseline_));
  }
  else
  {
    gzthrow(kPluginName << ": Invalid depth noise model: " << noise_model_name_);
  }
}

void GazeboNoisyDepthPlugin::advertise()
{
  ros::AdvertiseOptions depth_image_ao = ros::AdvertiseOptions::create<sensor_msgs::Image>(
    depth_image_topic_, 1, boost::bind(&GazeboNoisyDepthPlugin::depthImageConnect, this),
    boost::bind(&GazeboNoisyDepthPlugin::depthImageDisconnect, this), ros::VoidPtr(),
    &camera_queue_);

  depth_image_pub_ = rosnode_->advertise(depth_image_ao);

  ros::AdvertiseOptions depth_info_ao = ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    depth_info_topic_, 1, boost::bind(&GazeboNoisyDepthPlugin::depthInfoConnect, this),
    boost::bind(&GazeboNoisyDepthPlugin::depthInfoDisconnect, this), ros::VoidPtr(),
    &camera_queue_);

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

void GazeboNoisyDepthPlugin::fillDepthImage(const float* src)
{
  lock_.lock();

  // Copy data into image
  depth_image_msg_.header.frame_id = frame_name_;
  timeGazeboToRos(depth_sensor_update_time_, depth_image_msg_.header.stamp);

  // Copy from depth to depth image message
  if (fillDepthImageHelper(height_, width_, skip_, src, depth_image_msg_))
  {
    depth_image_pub_.publish(depth_image_msg_);
  }

  lock_.unlock();
}

bool GazeboNoisyDepthPlugin::fillDepthImageHelper(
  const uint32_t rows_arg,
  const uint32_t cols_arg,
  const uint32_t step_arg,
  const float* data_arg,
  sensor_msgs::Image& image_msg)
{
  if (data_arg == nullptr)
  {
    ROS_WARN_NAMED("NoisyDepth", "Invalid data array received - nullptr.");
    return false;
  }

  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.height = rows_arg;
  image_msg.width = cols_arg;
  image_msg.step = sizeof(float) * cols_arg;
  image_msg.data.resize(rows_arg * cols_arg * sizeof(float));
  image_msg.is_bigendian = 0;

  float* dest = (float*)(&(image_msg.data[0]));
  memcpy(dest, data_arg, sizeof(float) * width_ * height_);

  noise_model_->applyNoise(width_, height_, dest);

  return true;
}

void GazeboNoisyDepthPlugin::publishCameraInfo()
{
  // First publish parent camera info (IR camera)
  PublishCameraInfo();

  if (depth_info_connect_count_ <= 0)
  {
    return;
  }

  sensor_update_time_ = parentSensor_->LastMeasurementTime();
  common::Time cur_time = world_->SimTime();

  if (sensor_update_time_ - last_depth_info_update_time_ >= update_period_)
  {
    PublishCameraInfo(depth_info_pub_);
    last_depth_info_update_time_ = sensor_update_time_;
  }
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboNoisyDepthPlugin)
}  // namespace gazebo
