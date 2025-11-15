#include "tobas_img_processing/jpeg_compressor.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

#include <cstring>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::placeholders;

JPEGCompressor::JPEGCompressor(const rclcpp::NodeOptions& options)
  : tobas::BaseNode("jpeg_compressor", options)
{
  std::string jpeg_topic = getStringParam("jpeg_topic", std::string("image_compressed"));
  jpeg_sub_ = createSubscriber(jpeg_topic, &JPEGCompressor::callback, this);
  std::string jpeg_resized_topic = getStringParam("jpeg_resized_topic", std::string("jpeg_resized"));
  encoding_ = getStringParam("encoding", std::string("JPEG"));
  if (encoding_ == "JPEG"){
    jpeg_resized_pub_ = createPublisher<sensor_msgs::msg::CompressedImage>(jpeg_resized_topic);
  } else if (encoding_ == "H.264"){
    ffmpeg_packet_pub_ = createPublisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(jpeg_resized_topic);
    setFFmpegParameters();
  }
  else{
    RCLCPP_ERROR(this->get_logger(), "encoding %s is not supported. set JPEG or H.264.", encoding_.c_str());
  }
  resize_rate_ = getDoubleParam("resize_rate", 1.0);
}

void JPEGCompressor::setFFmpegParameters()
{
  encoder_.setEncoder("libx264");
  encoder_.setMeasurePerformance(false); // to suppress error "can't subtract times with different time sources [2 != 1]", somewhere else may be wrong?
  encoder_.addAVOption("tune", "zerolatency");
}

void JPEGCompressor::packetReady(
  const std::string & frame_id, const rclcpp::Time & stamp, const std::string & codec,
  uint32_t width, uint32_t height, uint64_t pts, uint8_t flags, uint8_t * data, size_t sz)
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

void JPEGCompressor::callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
{
  cv::Mat image = cv::imdecode(cv::Mat(msg->data), 1);
  cv::Mat image_resized ;
  if (resize_rate_ < 1.0){
    cv::resize(image, image_resized, cv::Size(), resize_rate_, resize_rate_); 
  }else{
    image_resized = image;
  }
  if (encoding_ == "JPEG"){
    auto message = std::make_unique<sensor_msgs::msg::CompressedImage>();
    message->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    message->header.frame_id = "map";
    message->format = std::string("jpeg");
    cv::imencode(".jpg", image_resized, message->data);
    jpeg_resized_pub_->publish(std::move(message));
  }else if (encoding_ == "H.264"){
    if (!this->encoder_.isInitialized()) {
      if (!this->encoder_.initialize(
            image_resized.cols, image_resized.rows,
            std::bind(&JPEGCompressor::packetReady, this, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
        RCLCPP_ERROR(this->get_logger(), "cannot initialize encoder!");
        return;
      }
    }
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_resized)
            .toImageMsg();
    encoder_.encodeImage(*message);
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(JPEGCompressor)
