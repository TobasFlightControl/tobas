#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_linux/video_dev.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

/**
 * @brief tobas_linux packageのVideoDev classを使ってuvcカメラを制御し，画像を取得してpublishする．
 */
class VideoDevPublisherNode : public tobas::BaseNode
{
  static constexpr int kFps = 30;

public:
  explicit VideoDevPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool initialize();
  void timerCallback();

  linux::VideoDev camera_;
  bool initialized_ = false;
  bool use_compressed_img_;
  std::string device_name_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_img_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

VideoDevPublisherNode::VideoDevPublisherNode(const rclcpp::NodeOptions& options)
  : tobas::BaseNode("video_dev_publisher", nodeOptions_Default(options))
{
  use_compressed_img_ = getBoolParam("use_compressed_image", true);
  device_name_ = getStringParam("device_name", "/dev/video0");
  const auto image_topic = getStringParam("image_topic", "image");

  if (use_compressed_img_) {
    compressed_img_publisher_ = createPublisher<sensor_msgs::msg::CompressedImage>(image_topic);
  }
  else {
    img_publisher_ = createPublisher<sensor_msgs::msg::Image>(image_topic);
  }

  timer_ = createTimer(std::chrono::milliseconds(1000 / kFps), &VideoDevPublisherNode::timerCallback, this);
}

bool VideoDevPublisherNode::initialize()
{
  if (use_compressed_img_) {
    if (!camera_.initialize(device_name_.c_str(), "MJPG")) {
      TOBAS_WARN("Failed to initialize camera.");
      return false;
    }
  }
  else {
    if (!camera_.initialize(device_name_.c_str(), "YUYV")) {
      TOBAS_WARN("Failed to initialize camera.");
      return false;
    }
  }

  if (!camera_.startStream()) {
    TOBAS_WARN("Failed to start stream.");
    return false;
  }

  TOBAS_INFO("Initialization succeed.");
  return true;
}

void VideoDevPublisherNode::timerCallback()
{
  if (!initialized_) {
    initialized_ = initialize();
  }

  if (!camera_.takePicture()) {
    TOBAS_WARN("Failed to take a picture.");
    return;
  }

  uint32_t image_size = 0;
  void* image_ptr = camera_.getImage(image_size);

  // using MJPG
  if (use_compressed_img_) {
    auto message_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
    message_compressed->header.stamp = this->now();
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
      TOBAS_WARN(
        "Current pixel format %u = %s is not supported yet", fmt.pixel_format, camera_.FCC2S(fmt.pixel_format).c_str());
    }
    img_publisher_->publish(std::move(message));
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(VideoDevPublisherNode)
