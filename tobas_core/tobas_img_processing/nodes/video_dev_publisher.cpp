#include "tobas_img_processing/video_dev_publisher.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

#include <cstring>

#include <opencv2/opencv.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

VideoDevPublisherNode::VideoDevPublisherNode(const rclcpp::NodeOptions& options)
  : tobas::BaseNode("video_dev_publisher", options)
{
  use_compressed_img_ = getBoolParam("use_compressed_image", true);
  device_name_ = getStringParam("device_name", std::string("/dev/video0"));
  if (use_compressed_img_) {
    std::string compressed_image_topic = getStringParam("compressed_image_topic", std::string("image_compressed"));
    compressed_img_publisher_ = createPublisher<sensor_msgs::msg::CompressedImage>(compressed_image_topic);
  }
  else {
    std::string image_topic = getStringParam("image_topic", std::string("image"));
    img_publisher_ = createPublisher<sensor_msgs::msg::Image>(image_topic);
  }
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / kFPS), std::bind(&VideoDevPublisherNode::timerCallback, this));
}

bool VideoDevPublisherNode::initialize()
{
  now_ = std::chrono::system_clock::now();
  last_send_ = now_;
  if (use_compressed_img_) {
    if (!camera_.initialize(device_name_.c_str(), "MJPG")) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize camera.");
      return false;
    }
  }
  else {
    if (!camera_.initialize(device_name_.c_str(), "YUYV")) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize camera.");
      return false;
    }
  }
  if (!camera_.startStream()) {
    RCLCPP_WARN(this->get_logger(), "Failed to start stream.");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Initialization succeed.");
  return true;
}

void VideoDevPublisherNode::timerCallback()
{
  now_ = std::chrono::system_clock::now();
  if (!initialized_) {
    initialized_ = initialize();
  }
  if (!camera_.takePicture()) {
    std::cerr << "Failed to take a picture." << std::endl;
    return;
  }
  uint image_size = 0;
  void* image_ptr = camera_.getImage(image_size);
  // using MJPG
  if (use_compressed_img_) {
    auto message_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
    message_compressed->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    message_compressed->header.frame_id = "map";
    message_compressed->format = std::string("jpeg");
    message_compressed->data.resize(image_size);
    std::memcpy(&*message_compressed->data.begin(), image_ptr, image_size);
    compressed_img_publisher_->publish(std::move(message_compressed));
  }
  else {
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    auto fmt = camera_.getImageFormat();
    message->width = fmt.width;
    message->height = fmt.height;
    message->step = fmt.bytes_per_line;
    message->data.resize(image_size);
    std::memcpy(&*message->data.begin(), image_ptr, image_size);
    if (fmt.pixel_format == V4L2_PIX_FMT_YUYV) {
      message->encoding = sensor_msgs::image_encodings::YUV422_YUY2;
    }
    else if (fmt.pixel_format == V4L2_PIX_FMT_UYVY) {
      message->encoding = sensor_msgs::image_encodings::YUV422;
    }
    else if (fmt.pixel_format == V4L2_PIX_FMT_GREY) {
      message->encoding = sensor_msgs::image_encodings::MONO8;
    }
    else {
      RCLCPP_WARN(
        this->get_logger(),
        "Current pixel format %u = %s is not supported yet",
        fmt.pixel_format,
        camera_.FCC2S(fmt.pixel_format).c_str());
    }
    img_publisher_->publish(std::move(message));
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(VideoDevPublisherNode)
