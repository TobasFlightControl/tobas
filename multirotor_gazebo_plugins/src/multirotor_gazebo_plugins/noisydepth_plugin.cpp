#include <sensor_msgs/image_encodings.h>

#include "../../include/multirotor_gazebo_plugins/noisydepth_plugin.hpp"
#include "../../include/multirotor_gazebo_plugins/utils.hpp"

using namespace std;

namespace gazebo
{
GazeboNoisyDepthPlugin::GazeboNoisyDepthPlugin()
{
  depth_info_connect_count_ = 0;
  depth_image_connect_count_ = 0;
  last_depth_info_update_time_ = common::Time(0);
}

void GazeboNoisyDepthPlugin::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
  DepthCameraPlugin::Load(parent, sdf);

  // Copy from DepthCameraPlugin into GazeboRosCameraUtils
  parentSensor_ = parentSensor;
  width_ = width;
  height_ = height;
  depth_ = depth;
  format_ = format;
  camera_ = depthCamera;

  GazeboRosCameraUtils::Load(parent, sdf);

  getSdfParams(sdf);
  setNoiseModel();

  // Listen to the update event
  update_connection_ =
    GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboNoisyDepthPlugin::advertise, this));
}

void GazeboNoisyDepthPlugin::OnNewDepthFrame(
  const float* image,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  const string& format)
{
  if (!initialized_ || height_ <= 0 || width_ <= 0)
  {
    return;
  }

  depth_sensor_update_time_ = parentSensor->LastMeasurementTime();

  // Check if there are subscribers, if not disable parent, else process images.
  if (parentSensor->IsActive())
  {
    if (depth_image_connect_count_ <= 0 && (*image_connect_count_) <= 0)
    {
      parentSensor->SetActive(false);
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
      parentSensor->SetActive(true);
    }
  }

  publishCameraInfo();
}

void GazeboNoisyDepthPlugin::OnNewImageFrame(
  const u_char* image,
  uint32_t width,
  uint32_t height,
  uint32_t depth,
  const string& format)
{
  if (!initialized_ || height_ <= 0 || width_ <= 0)
  {
    return;
  }

  sensor_update_time_ = parentSensor_->LastMeasurementTime();

  // Check if there are subscribers, if not disable parent, else process images..
  if (parentSensor->IsActive())
  {
    if (depth_image_connect_count_ <= 0 && (*image_connect_count_) <= 0)
    {
      parentSensor->SetActive(false);
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
      parentSensor->SetActive(true);
    }
  }
}

void GazeboNoisyDepthPlugin::getSdfParams(sdf::ElementPtr sdf)
{
  if (!getSdfParam<string>(sdf, "robotNamespace", ns_))
  {
    gzthrow(kPluginName << ": Please specify a robotNamespace.");
  }

  getSdfParam<string>(sdf, "irImageTopic", image_topic_name_, kDefaultIrImageTopic);
  getSdfParam<string>(sdf, "irInfoTopic", camera_info_topic_name_, kDefaultIrInfoTopic);
  getSdfParam<string>(sdf, "depthImageTopic", depth_image_topic_, kDefaultIrImageTopic);
  getSdfParam<string>(sdf, "depthInfoTopic", depth_info_topic_, kDefaultIrInfoTopic);
  getSdfParam<string>(sdf, "depthNoiseModel", noise_model_name_, kDefaultIrInfoTopic);
  getSdfParam<float>(sdf, "depthNoiseMinDist", noise_min_dist_, kDefaultDepthNoiseMinDist);
  getSdfParam<float>(sdf, "depthNoiseMaxDist", noise_max_dist_, kDefaultDepthNoiseMaxDist);
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
    noise_model_.reset(new D435DepthNoiseModel(noise_min_dist_, noise_max_dist_));
  }
  else
  {
    gzthrow(kPluginName << ": Invalid depth noise model: " << noise_model_name_);
  }
}

void GazeboNoisyDepthPlugin::advertise()
{
  ros::AdvertiseOptions depth_image_ao = ros::AdvertiseOptions::create<sensor_msgs::Image>(
    "/" + ns_ + "/" + depth_image_topic_, 1,
    boost::bind(&GazeboNoisyDepthPlugin::depthImageConnect, this),
    boost::bind(&GazeboNoisyDepthPlugin::depthImageDisconnect, this), ros::VoidPtr(),
    &camera_queue_);

  depth_image_pub_ = rosnode_->advertise(depth_image_ao);

  ros::AdvertiseOptions depth_info_ao = ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    "/" + ns_ + "/" + depth_info_topic_, 1,
    boost::bind(&GazeboNoisyDepthPlugin::depthInfoConnect, this),
    boost::bind(&GazeboNoisyDepthPlugin::depthInfoDisconnect, this), ros::VoidPtr(),
    &camera_queue_);

  depth_info_pub_ = rosnode_->advertise(depth_info_ao);
}

void GazeboNoisyDepthPlugin::depthImageConnect()
{
  ++depth_image_connect_count_;
  parentSensor->SetActive(true);
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
  depth_image_msg_.header.stamp.sec = depth_sensor_update_time_.sec;
  depth_image_msg_.header.stamp.nsec = depth_sensor_update_time_.nsec;

  // Copy from depth to depth image message
  if (fillDepthImageHelper(height, width, skip_, src, depth_image_msg_))
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
  memcpy(dest, data_arg, sizeof(float) * width * height);

  noise_model_->applyNoise(width, height, dest);

  return true;
}

void GazeboNoisyDepthPlugin::publishCameraInfo()
{
  // First publish parent camera info (IR camera)
  GazeboRosCameraUtils::PublishCameraInfo();

  if (depth_info_connect_count_ <= 0)
  {
    return;
  }

  sensor_update_time_ = parentSensor_->LastMeasurementTime();
  common::Time cur_time = world_->SimTime();

  if (sensor_update_time_ - last_depth_info_update_time_ >= update_period_)
  {
    GazeboRosCameraUtils::PublishCameraInfo(depth_info_pub_);
    last_depth_info_update_time_ = sensor_update_time_;
  }
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboNoisyDepthPlugin)
}  // namespace gazebo
