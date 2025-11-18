#include "tobas_img_processing/cx_gb400_publisher.hpp"

#include <linux/videodev2.h>

#include <cstring>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

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
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / fps), std::bind(&CxGb400PublisherNode::timerCallback, this));
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
  now_ = std::chrono::system_clock::now();
  last_send_ = now_;
  if (!camera_.initialize(device_name_.c_str(), camera_.kLower, true, disable_video_streaming_)) {
    RCLCPP_WARN(this->get_logger(), "Failed to initialize camera.");
    return false;
  }
  if (!disable_video_streaming_){
    if (!camera_.startStream()) {
      RCLCPP_WARN(this->get_logger(), "Failed to start stream.");
      return false;
    }
  }
  if (!camera_.sendGimbalCtrl(0.0, 0.0)) {
    RCLCPP_WARN(this->get_logger(), "Failed to send gimbal control angles.");
    return false;
  }
  return true;
}

void CxGb400PublisherNode::timerCallback()
{
  now_ = std::chrono::system_clock::now();
  if (!initialized_) {
    initialized_ = initialize();
  }
  if (std::chrono::duration_cast<std::chrono::milliseconds>(now_ - last_send_) > camera_.kSendAttitudeInterval) {
    if (!camera_.sendCopterAttitude(
          copter_attitude_.w(), copter_attitude_.x(), copter_attitude_.y(), copter_attitude_.z())) {
      RCLCPP_WARN(this->get_logger(), "Failed to send copter attitude.");
      return;
    }
    last_send_ = now_;
  }
  if (!disable_video_streaming_){
    // take a picture
    if (!camera_.takePicture()) {
      std::cerr << "Failed to take a picture." << std::endl;
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
        RCLCPP_ERROR(this->get_logger(), "cannot initialize encoder!");
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
    RCLCPP_WARN(this->get_logger(), "Failed to send gimbal control command.");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(CxGb400PublisherNode)
