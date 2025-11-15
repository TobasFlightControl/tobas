#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <eigen3/Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <tobas_ic_drivers/cx_gb400.hpp>
#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>

#include <tobas_msgs/msg/gimbal_attitude_command.hpp>
#include <tobas_msgs_adapter/odometry.hpp>

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
  std::chrono::system_clock::time_point last_send_, now_;
  bool initialized_ = false;
  bool disable_video_streaming_ = false;
  std::string device_name_;
  ffmpeg_encoder_decoder::Encoder encoder_;
};
