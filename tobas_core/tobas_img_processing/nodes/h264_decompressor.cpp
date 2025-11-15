#include "tobas_img_processing/h264_decompressor.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

#include <cstring>

#include <opencv2/opencv.hpp>

#include <ffmpeg_encoder_decoder/utils.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using namespace std::placeholders;

H264Decompressor::H264Decompressor(const rclcpp::NodeOptions& options)
  : tobas::BaseNode("h264_decompressor", options)
{
  std::string h264_topic = getStringParam("h264_topic", std::string("image_h264"));
  h264_sub_ = createSubscriber(h264_topic, &H264Decompressor::callback, this);
  std::string image_raw_topic = getStringParam("decoded_topic", std::string("image_h264_decoded"));
  raw_img_pub_ = createPublisher<sensor_msgs::msg::Image>(image_raw_topic);
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
      msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id,
      msg->header.stamp);
    return;
  }
  // need to initialize the decoder
  if (msg->flags == 0) {
    return;  // wait for key frame!
  }
  if (msg->encoding.empty()) {
    RCLCPP_ERROR(this->get_logger(), "no encoding provided!");
    return;
  }
  if (!decoder_.initialize(msg->encoding, std::bind(&H264Decompressor::publishRawImg, this, _1), "h264")){
    RCLCPP_ERROR(this->get_logger(), "cannot initialize decoder: h264");
    return;
  }
  decoder_.decodePacket(
    msg->encoding, &msg->data[0], msg->data.size(), msg->pts, msg->header.frame_id,
    msg->header.stamp);
}

RCLCPP_COMPONENTS_REGISTER_NODE(H264Decompressor)
