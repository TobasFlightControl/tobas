#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tobas_linux/video_dev.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @brief tobas_linux packageのVideoDev classを使ってuvcカメラを制御し，画像を取得してpublishする．
 *
 */
class VideoDevPublisherNode : public tobas::BaseNode
{
  static constexpr int kFPS = 30;

public:
  explicit VideoDevPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool initialize();
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_img_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
  linux::VideoDev camera_;
  std::chrono::system_clock::time_point last_send_, now_;
  bool initialized_ = false;
  bool use_compressed_img_;
  std::string device_name_;

  std::string FCC2S(const unsigned int& val);
};
