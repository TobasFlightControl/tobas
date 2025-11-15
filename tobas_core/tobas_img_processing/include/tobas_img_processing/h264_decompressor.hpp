#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ffmpeg_encoder_decoder/decoder.hpp>
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
class H264Decompressor : public tobas::BaseNode
{
public:
  explicit H264Decompressor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool initialize();
  void publishRawImg(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void callback(const ffmpeg_image_transport_msgs::msg::FFMPEGPacket::ConstSharedPtr& msg);
  rclcpp::Subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr h264_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_img_pub_;
  bool initialized_ = false;
  ffmpeg_encoder_decoder::Decoder decoder_;
};
