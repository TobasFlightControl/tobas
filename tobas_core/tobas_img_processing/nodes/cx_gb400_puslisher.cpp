#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <linux/videodev2.h>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_ic_drivers/cx_gb400.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>

#include <tobas_msgs/msg/gimbal_attitude_command.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class CxGb400PublisherNode : public tobas::BaseNode
{
public:
  explicit CxGb400PublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool initialize();
  void setFFmpegParameters();
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
  void timerCallback();
  void copterAttMsgCb(const tobas_msgs::Odometry::ConstSharedPtr& _msg);
  void gimbalAttitudeCmdCb(const tobas_msgs::msg::GimbalAttitudeCommand::ConstSharedPtr& _msg);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr ffmpeg_packet_pub_;
  rclcpp::Subscription<tobas_msgs::Odometry>::SharedPtr copter_att_sub_;
  rclcpp::Subscription<tobas_msgs::msg::GimbalAttitudeCommand>::SharedPtr gimbal_att_cmd_sub_;
  driver::CxGb400 camera_;
  Eigen::Quaterniond copter_attitude_ = Eigen::Quaterniond::Identity();
  rclcpp::Time last_send_;
  bool initialized_ = false;
  bool disable_video_streaming_ = false;
  std::string device_name_;
  ffmpeg_encoder_decoder::Encoder encoder_;
};

CxGb400PublisherNode::CxGb400PublisherNode(const rclcpp::NodeOptions& options)
  : tobas::BaseNode("cx_gb400_publisher", options)
{
  device_name_ = getStringParam("device_name", std::string("/dev/video0"));
  disable_video_streaming_ = getBoolParam("disable_video_streaming", false);
  copter_att_sub_ = createSubscriber("odom", &CxGb400PublisherNode::copterAttMsgCb, this);
  gimbal_att_cmd_sub_ = createSubscriber("gimbal_attitude_command", &CxGb400PublisherNode::gimbalAttitudeCmdCb, this);
  if (!disable_video_streaming_){
    std::string h264_topic = getStringParam("image_topic", std::string("image"));
    ffmpeg_packet_pub_ = createPublisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(h264_topic);
    setFFmpegParameters();
  }
  int fps = getIntParam("FPS", 30);
  timer_ = this->createTimer(std::chrono::milliseconds(1000 / fps), &CxGb400PublisherNode::timerCallback, this);
  last_send_ = this->now();
}

void CxGb400PublisherNode::setFFmpegParameters()
{
  encoder_.setEncoder("libx264");
  encoder_.setMeasurePerformance(false); // to suppress error "can't subtract times with different time sources [2 != 1]", somewhere else may be wrong?
  encoder_.addAVOption("tune", "zerolatency");
}

void CxGb400PublisherNode::packetReady(
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

bool CxGb400PublisherNode::initialize()
{
  if (!camera_.initialize(device_name_.c_str(), camera_.kLower, true, disable_video_streaming_)) {
    TOBAS_WARN("Failed to initialize camera.");
    return false;
  }
  if (!disable_video_streaming_){
    if (!camera_.startStream()) {
      TOBAS_WARN("Failed to start stream.");
      return false;
    }
  }
  if (!camera_.sendGimbalCtrl(0.0, 0.0)) {
    TOBAS_WARN("Failed to send gimbal control angles.");
    return false;
  }
  return true;
}

void CxGb400PublisherNode::timerCallback()
{
  rclcpp::Time now = this->now();
  if (!initialized_) {
    initialized_ = initialize();
  }
  if ((now - last_send_).to_chrono<std::chrono::milliseconds>() > camera_.kSendAttitudeInterval) {
    if (!camera_.sendCopterAttitude(
          copter_attitude_.w(), copter_attitude_.x(), copter_attitude_.y(), copter_attitude_.z())) {
      TOBAS_WARN("Failed to send copter attitude.");
      return;
    }
    last_send_ = now;
  }
  if (!disable_video_streaming_){
    // take a picture
    if (!camera_.takePicture()) {
      TOBAS_WARN("Failed to take a picture.");
      return;
    }
    uint image_size = 0;
    void* image_ptr = camera_.getImage(image_size);
    std::vector<uint8_t> image_data(image_size);
    std::memcpy(&*image_data.begin(), image_ptr, image_size);
    cv::Mat image = cv::imdecode(cv::Mat(image_data), 1);
    if (!this->encoder_.isInitialized()) {
      if (!this->encoder_.initialize(
            image.cols, image.rows,
            std::bind(&CxGb400PublisherNode::packetReady, this, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
        TOBAS_ERROR("Cannot initialize encoder!");
        return;
      }
    }
    auto message = std::make_shared<sensor_msgs::msg::Image>();
    message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
            .toImageMsg();
    encoder_.encodeImage(*message);
  }
}

void CxGb400PublisherNode::copterAttMsgCb(const tobas_msgs::Odometry::ConstSharedPtr& _msg)
{
  copter_attitude_ = Eigen::Quaterniond(_msg->frame.M.data);
}

void CxGb400PublisherNode::gimbalAttitudeCmdCb(const tobas_msgs::msg::GimbalAttitudeCommand::ConstSharedPtr& _msg)
{
  if (!camera_.sendGimbalCtrl(_msg->pitch, _msg->yaw)) {
    TOBAS_WARN("Failed to send gimbal control command.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(CxGb400PublisherNode)
