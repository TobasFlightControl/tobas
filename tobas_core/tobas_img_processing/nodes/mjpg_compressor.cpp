#include <fcntl.h>
#include <linux/videodev2.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <tobas_linux/video_dev.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::placeholders;

/**
 * @brief sensor_msgs/msg/CompressedImage型の画像をsubscribeして，データサイズを落としたあとpublishする．
 */
class MjpgCompressor : public tobas::BaseNode
{
public:
  explicit MjpgCompressor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr mjpg_sub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr mjpg_resized_pub_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr ffmpeg_packet_pub_;
  bool initialized_ = false;
  // publishする画像のencoding．MJPGか，H.264か．
  std::string encoding_;
  // resizeする割合
  double resize_rate_ = 1.0;
  // H.264用encoder
  ffmpeg_encoder_decoder::Encoder encoder_;
};

MjpgCompressor::MjpgCompressor(const rclcpp::NodeOptions& options) : tobas::BaseNode("mjpg_compressor", options)
{
  std::string mjpg_topic = getStringParam("mjpg_topic", std::string("image_compressed"));
  mjpg_sub_ = createSubscriber(mjpg_topic, &MjpgCompressor::callback, this);
  std::string resized_topic = getStringParam("resized_topic", std::string("mjpg_resized"));
  encoding_ = getStringParam("encoding", std::string("MJPG"));
  if (encoding_ == "MJPG") {
    mjpg_resized_pub_ = createPublisher<sensor_msgs::msg::CompressedImage>(resized_topic);
  }
  else if (encoding_ == "H.264") {
    ffmpeg_packet_pub_ = createPublisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(resized_topic);
    setFFmpegParameters();
  }
  else {
    TOBAS_ERROR("encoding %s is not supported. set MJPG or H.264.", encoding_.c_str());
  }
  resize_rate_ = getDoubleParam("resize_rate", 1.0);
}

void MjpgCompressor::setFFmpegParameters()
{
  encoder_.setEncoder("libx264");
  encoder_.setMeasurePerformance(
    false);  // to suppress error "can't subtract times with different time sources [2 != 1]", somewhere else may be wrong?
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
    message->header.stamp = this->now();
    message->header.frame_id = "map";
    message->format = std::string("jpeg");
    cv::imencode(".jpg", image_resized, message->data);
    mjpg_resized_pub_->publish(std::move(message));
  }
  else if (encoding_ == "H.264") {
    if (!this->encoder_.isInitialized()) {
      if (!this->encoder_.initialize(
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

RCLCPP_COMPONENTS_REGISTER_NODE(MjpgCompressor)
