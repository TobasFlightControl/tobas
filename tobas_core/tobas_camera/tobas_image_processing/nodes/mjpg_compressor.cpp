// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

#include <cv_bridge/cv_bridge.hpp>
#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

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
 * @brief Subscribe to sensor_msgs/msg/CompressedImage images, reduce their data size, and publish the result.
 */
class MjpgCompressor : public BaseNode
{
public:
  explicit MjpgCompressor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void setFfmpegParameters();
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

  std::string encoding_;      // Encoding of the published image, either MJPG or H.264.
  double resize_rate_ = 1.0;  // Resize ratio

  bool initialized_ = false;
  ffmpeg_encoder_decoder::Encoder encoder_;  // H.264 encoder

  ros2::PublisherPtr<sensor_msgs::msg::CompressedImage> mjpg_resized_pub_;
  ros2::PublisherPtr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> ffmpeg_packet_pub_;
  ros2::SubscriberPtr<sensor_msgs::msg::CompressedImage> mjpg_sub_;
};

MjpgCompressor::MjpgCompressor(const rclcpp::NodeOptions& options)
  : BaseNode("mjpg_compressor", nodeOptions_Default(options))
{
  const auto mjpg_topic = getStringParam("mjpg_topic", "image_compressed");
  const auto resized_topic = getStringParam("resized_topic", "mjpg_resized");
  encoding_ = getStringParam("encoding", "MJPG");
  resize_rate_ = getDoubleParam("resize_rate", 1.0);

  if (encoding_ == "MJPG") {
    mjpg_resized_pub_ = createPublisher<sensor_msgs::msg::CompressedImage>(resized_topic);
  }
  else if (encoding_ == "H.264") {
    ffmpeg_packet_pub_ = createPublisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(resized_topic);
    setFfmpegParameters();
  }
  else {
    TOBAS_ERROR("encoding ", encoding_, " is not supported. set MJPG or H.264.");
    return;
  }

  mjpg_sub_ = createSubscriber(mjpg_topic, &MjpgCompressor::callback, this);
}

void MjpgCompressor::setFfmpegParameters()
{
  encoder_.setEncoder("libx264");
  // Disable performance measurement to avoid subtracting timestamps from different time sources.
  encoder_.setMeasurePerformance(false);
  encoder_.addAVOption("tune", "zerolatency");
}

void MjpgCompressor::packetReady(
  const std::string& frame_id,
  const rclcpp::Time& stamp,
  const std::string& codec,
  uint32_t width,
  uint32_t height,
  uint64_t pts,
  uint8_t flags,
  uint8_t* data,
  size_t sz)
{
  auto msg = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
  msg->header.frame_id = frame_id;
  msg->header.stamp = stamp;
  msg->encoding = codec;
  msg->width = width;
  msg->height = height;
  msg->pts = pts;
  msg->flags = flags;
  msg->data.assign(data, data + sz);
  ffmpeg_packet_pub_->publish(std::move(msg));
}

void MjpgCompressor::callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
{
  cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);
  cv::Mat image_resized;
  if (resize_rate_ < 1.0) {
    cv::resize(image, image_resized, cv::Size(), resize_rate_, resize_rate_);
  }
  else {
    image_resized = image;
  }
  if (encoding_ == "MJPG") {
    auto message = std::make_unique<sensor_msgs::msg::CompressedImage>();
    message->header.stamp = now();
    message->header.frame_id = "map";
    message->format = std::string("jpeg");
    cv::imencode(".jpg", image_resized, message->data);
    mjpg_resized_pub_->publish(std::move(message));
  }
  else if (encoding_ == "H.264") {
    if (!encoder_.isInitialized()) {
      if (!encoder_.initialize(
            image_resized.cols,
            image_resized.rows,
            std::bind(&MjpgCompressor::packetReady, this, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
        TOBAS_ERROR("Cannot initialize encoder!");
        return;
      }
    }
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_resized).toImageMsg();
    encoder_.encodeImage(*message);
  }
}
}  // namespace camera
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::camera::MjpgCompressor)
