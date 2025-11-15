#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tobas_linux/video_dev.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

/**
 * @brief sensor_msgs/msg/CompressedImage型の画像をsubscribeして，データサイズを落としたあとpublishする．
 *
 */
class JPEGCompressor : public tobas::BaseNode
{

public:
  explicit JPEGCompressor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool initialize();
  void setFFmpegParameters();
  void handleAVOptions(const std::string& opt);
  void packetReady(
    const std::string& frame_id,
    const rclcpp::Time& stamp,
    const std::string& codec,
    uint32_t width,
    uint32_t height,
    uint64_t pts,
    uint8_t flags,
    uint8_t* data,
    size_t sz);
  void callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr jpeg_resized_pub_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr ffmpeg_packet_pub_;
  bool initialized_ = false;
  // publishする画像のencoding．BGR8か，JPEGか，H.264か．
  std::string encoding_;
  // resizeする割合
  double resize_rate_ = 1.0;
  // H.264用encoder
  ffmpeg_encoder_decoder::Encoder encoder_;
};
