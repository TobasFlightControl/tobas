// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <ffmpeg_encoder_decoder/decoder.hpp>
#include <ffmpeg_encoder_decoder/utils.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <opencv2/opencv.hpp>

#include <tobas_linux/video_dev.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::placeholders;

namespace tobas
{
namespace camera
{
/**
 * @brief Subscribe to H.264-compressed ffmpeg_image_transport_msgs/msg/FFMPEGPacket images, decompress them, and publish them.
 */
class H264Decompressor : public BaseNode
{
public:
  explicit H264Decompressor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void publishRawImg(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void callback(const ffmpeg_image_transport_msgs::msg::FFMPEGPacket::ConstSharedPtr& msg);

  ros2::SubscriberPtr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> h264_sub_;
  ros2::PublisherPtr<sensor_msgs::msg::Image> raw_img_pub_;

  bool initialized_ = false;
  ffmpeg_encoder_decoder::Decoder decoder_;
};

H264Decompressor::H264Decompressor(const rclcpp::NodeOptions& options)
  : BaseNode("h264_decompressor", nodeOptions_Default(options))
{
  const auto image_raw_topic = getStringParam("decoded_topic", "image_h264_decoded");
  const auto h264_topic = getStringParam("h264_topic", "image_h264");

  raw_img_pub_ = createPublisher<sensor_msgs::msg::Image>(image_raw_topic);
  h264_sub_ = createSubscriber(h264_topic, &H264Decompressor::callback, this);
}

void H264Decompressor::publishRawImg(const sensor_msgs::msg::Image::ConstSharedPtr& msg_ptr)
{
  raw_img_pub_->publish(*msg_ptr);
}

void H264Decompressor::callback(const ffmpeg_image_transport_msgs::msg::FFMPEGPacket::ConstSharedPtr& msg)
{
  if (decoder_.isInitialized()) {
    // the decoder is already initialized
    decoder_.decodePacket(
      msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id, msg->header.stamp);
    return;
  }

  // need to initialize the decoder
  if (msg->flags == 0) {
    return;  // wait for key frame!
  }

  if (msg->encoding.empty()) {
    TOBAS_ERROR("no encoding provided!");
    return;
  }

  if (!decoder_.initialize(msg->encoding, std::bind(&H264Decompressor::publishRawImg, this, _1), "h264")) {
    TOBAS_ERROR("cannot initialize decoder: h264");
    return;
  }

  decoder_.decodePacket(
    msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id, msg->header.stamp);
}
}  // namespace camera
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::camera::H264Decompressor)
