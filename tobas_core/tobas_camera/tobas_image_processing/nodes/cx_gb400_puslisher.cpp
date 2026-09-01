// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <linux/videodev2.h>

#include <cv_bridge/cv_bridge.hpp>
#include <eigen3/Eigen/Geometry>
#include <ffmpeg_encoder_decoder/encoder.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

#include <tobas_camera_ros_interface/ros_interface.hpp>
#include <tobas_ic_drivers/cx_gb400.hpp>
#include <tobas_node/node.hpp>

#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <tobas_camera_msgs/msg/gimbal_attitude_command.hpp>
#include <tobas_camera_msgs/msg/status.hpp>
#include <tobas_camera_msgs/srv/format_sd_card.hpp>
#include <tobas_camera_msgs/srv/set_photo_quality.hpp>
#include <tobas_camera_msgs/srv/set_video_frame_rate.hpp>
#include <tobas_camera_msgs/srv/set_video_quality.hpp>
#include <tobas_camera_msgs/srv/start_recording.hpp>
#include <tobas_camera_msgs/srv/stop_recording.hpp>
#include <tobas_camera_msgs/srv/take_picture_to_sd.hpp>
#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace tobas
{
namespace camera
{
class CxGb400PublisherNode : public BaseNode
{
public:
  static constexpr auto kStatusSendInterval = 1s;

  explicit CxGb400PublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool initialize();
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

  void timerCallback();

  void copterAttMsgCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _msg);
  void gimbalAttitudeCmdCb(const tobas_camera_msgs::msg::GimbalAttitudeCommand::ConstSharedPtr& _msg);
  void formatSdCardCb(
    const tobas_camera_msgs::srv::FormatSdCard::Request::ConstSharedPtr&,
    const tobas_camera_msgs::srv::FormatSdCard::Response::SharedPtr& res);
  void setPhotoQualityCb(
    const tobas_camera_msgs::srv::SetPhotoQuality::Request::ConstSharedPtr& req,
    const tobas_camera_msgs::srv::SetPhotoQuality::Response::SharedPtr& res);
  void setVideoFrameRateCb(
    const tobas_camera_msgs::srv::SetVideoFrameRate::Request::ConstSharedPtr& req,
    const tobas_camera_msgs::srv::SetVideoFrameRate::Response::SharedPtr& res);
  void setVideoQualityCb(
    const tobas_camera_msgs::srv::SetVideoQuality::Request::ConstSharedPtr& req,
    const tobas_camera_msgs::srv::SetVideoQuality::Response::SharedPtr& res);
  void startRecordingCb(
    const tobas_camera_msgs::srv::StartRecording::Request::ConstSharedPtr&,
    const tobas_camera_msgs::srv::StartRecording::Response::SharedPtr& res);
  void stopRecordingCb(
    const tobas_camera_msgs::srv::StopRecording::Request::ConstSharedPtr&,
    const tobas_camera_msgs::srv::StopRecording::Response::SharedPtr& res);
  void takePictureToSdCb(
    const tobas_camera_msgs::srv::TakePictureToSd::Request::ConstSharedPtr&,
    const tobas_camera_msgs::srv::TakePictureToSd::Response::SharedPtr& res);

  // Parameters
  std::string device_name_;
  bool disable_video_streaming_;

  ros2::TimerPtr timer_;

  ros2::PublisherPtr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> ffmpeg_packet_pub_;
  ros2::PublisherPtr<tobas_camera_msgs::msg::Status> camera_status_pub_;
  ros2::SubscriberPtr<tobas_msgs::OdometryWithCovarianceStamped> copter_att_sub_;
  ros2::SubscriberPtr<tobas_camera_msgs::msg::GimbalAttitudeCommand> gimbal_att_cmd_sub_;
  ros2::ServiceServerPtr<tobas_camera_msgs::srv::FormatSdCard> format_sd_card_ss_;
  ros2::ServiceServerPtr<tobas_camera_msgs::srv::SetPhotoQuality> set_photo_quality_ss_;
  ros2::ServiceServerPtr<tobas_camera_msgs::srv::SetVideoFrameRate> set_video_frame_rate_ss_;
  ros2::ServiceServerPtr<tobas_camera_msgs::srv::SetVideoQuality> set_video_quality_ss_;
  ros2::ServiceServerPtr<tobas_camera_msgs::srv::StartRecording> start_recording_ss_;
  ros2::ServiceServerPtr<tobas_camera_msgs::srv::StopRecording> stop_recording_ss_;
  ros2::ServiceServerPtr<tobas_camera_msgs::srv::TakePictureToSd> take_picture_to_sd_ss_;

  driver::CxGb400 camera_;
  Eigen::Quaterniond copter_attitude_ = Eigen::Quaterniond::Identity();
  rclcpp::Time last_attitude_send_;
  rclcpp::Time last_status_send_;
  bool initialized_ = false;
  ffmpeg_encoder_decoder::Encoder encoder_;
};

CxGb400PublisherNode::CxGb400PublisherNode(const rclcpp::NodeOptions& options)
  : BaseNode("cx_gb400_publisher", nodeOptions_Default(options))
{
  device_name_ = getStringParam("device_name", "/dev/video0");
  disable_video_streaming_ = getBoolParam("disable_video_streaming", false);
  const auto fps = getIntParam("FPS", 30);

  if (!disable_video_streaming_) {
    const auto h264_topic = getStringParam("image_topic", "image");
    ffmpeg_packet_pub_ = createPublisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(h264_topic);
    setFfmpegParameters();
  }
  camera_status_pub_ = createPublisher<tobas_camera_msgs::msg::Status>(topic::kCameraStatus);

  copter_att_sub_ = createSubscriber(tobas::topic::kOdometry, &CxGb400PublisherNode::copterAttMsgCb, this);
  gimbal_att_cmd_sub_ = createSubscriber(topic::kGimbalAttitudeCmd, &CxGb400PublisherNode::gimbalAttitudeCmdCb, this);

  format_sd_card_ss_ = createService<tobas_camera_msgs::srv::FormatSdCard>(
    service::kFormatSdCard, &CxGb400PublisherNode::formatSdCardCb, this);
  set_photo_quality_ss_ = createService<tobas_camera_msgs::srv::SetPhotoQuality>(
    service::kSetPhotoQuality, &CxGb400PublisherNode::setPhotoQualityCb, this);
  set_video_frame_rate_ss_ = createService<tobas_camera_msgs::srv::SetVideoFrameRate>(
    service::kSetVideoFrameRate, &CxGb400PublisherNode::setVideoFrameRateCb, this);
  set_video_quality_ss_ = createService<tobas_camera_msgs::srv::SetVideoQuality>(
    service::kSetVideoQuality, &CxGb400PublisherNode::setVideoQualityCb, this);
  start_recording_ss_ = createService<tobas_camera_msgs::srv::StartRecording>(
    service::kStartRecording, &CxGb400PublisherNode::startRecordingCb, this);
  stop_recording_ss_ = createService<tobas_camera_msgs::srv::StopRecording>(
    service::kStopRecording, &CxGb400PublisherNode::stopRecordingCb, this);
  take_picture_to_sd_ss_ = createService<tobas_camera_msgs::srv::TakePictureToSd>(
    service::kTakePictureToSd, &CxGb400PublisherNode::takePictureToSdCb, this);

  timer_ = createTimer(std::chrono::milliseconds(1000 / fps), &CxGb400PublisherNode::timerCallback, this);
  last_attitude_send_ = now();
  last_status_send_ = now();
}

void CxGb400PublisherNode::setFfmpegParameters()
{
  encoder_.setEncoder("libx264");
  // To suppress error "can't subtract times with different time sources [2 != 1]", somewhere else may be wrong?
  encoder_.setMeasurePerformance(false);
  encoder_.addAVOption("tune", "zerolatency");
}

void CxGb400PublisherNode::packetReady(
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

bool CxGb400PublisherNode::initialize()
{
  if (!camera_.initialize(
        device_name_.c_str(), driver::CxGb400::CameraPosition::kLower, true, disable_video_streaming_)) {
    TOBAS_WARN("Failed to initialize camera.");
    return false;
  }
  if (!disable_video_streaming_) {
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
  const auto now = this->now();
  if (!initialized_) {
    initialized_ = initialize();
  }

  // Send the multicopter attitude to the camera.
  if ((now - last_attitude_send_).to_chrono<std::chrono::milliseconds>() >= driver::CxGb400::kSendAttitudeInterval) {
    if (camera_.sendCopterAttitude(
          copter_attitude_.w(), copter_attitude_.x(), copter_attitude_.y(), copter_attitude_.z())) {
      last_attitude_send_ = now;
    }
    else {
      TOBAS_WARN("Failed to send copter attitude.");
    }
  }

  // Read and publish the camera status.
  if ((now - last_status_send_).to_chrono<std::chrono::seconds>() >= kStatusSendInterval) {
    auto msg = std::make_unique<tobas_camera_msgs::msg::Status>();
    if (camera_.getCameraStatus(
          msg->sd_full,
          msg->time_not_set,
          msg->media_error,
          msg->lens_error,
          msg->gimbal_error,
          msg->gimbal_motor_error,
          msg->gimbal_control_error,
          msg->thermal_error,
          msg->video_remain_time,
          msg->photo_remain_count,
          msg->card_full_size,
          msg->card_free_mem,
          msg->aperture,
          msg->iso)) {
      camera_status_pub_->publish(std::move(msg));
      last_status_send_ = now;
    }
    else {
      TOBAS_WARN("Failed to get camera status");
    }
  }

  // Publish HVideo images.
  if (!disable_video_streaming_) {
    // Capture an image.
    if (!camera_.takePicture()) {
      TOBAS_WARN("Failed to take a picture.");
      return;
    }
    uint32_t image_size;
    void* image_ptr = camera_.getImage(image_size);
    std::vector<uint8_t> image_data(image_size);
    std::memcpy(&*image_data.begin(), image_ptr, image_size);
    const auto image = cv::imdecode(cv::Mat(image_data), 1);
    if (!encoder_.isInitialized()) {
      if (!encoder_.initialize(
            image.cols,
            image.rows,
            std::bind(&CxGb400PublisherNode::packetReady, this, _1, _2, _3, _4, _5, _6, _7, _8, _9))) {
        TOBAS_ERROR("Cannot initialize encoder!");
        return;
      }
    }
    const auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    encoder_.encodeImage(*message);
  }
}

void CxGb400PublisherNode::copterAttMsgCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _msg)
{
  copter_attitude_ = Eigen::Quaterniond(_msg->odom.odom.frame.M.data);
}

void CxGb400PublisherNode::gimbalAttitudeCmdCb(const tobas_camera_msgs::msg::GimbalAttitudeCommand::ConstSharedPtr& _msg)
{
  if (!camera_.sendGimbalCtrl(_msg->pitch, _msg->yaw)) {
    TOBAS_WARN("Failed to send gimbal control command.");
  }
}

void CxGb400PublisherNode::formatSdCardCb(
  const tobas_camera_msgs::srv::FormatSdCard::Request::ConstSharedPtr&,
  const tobas_camera_msgs::srv::FormatSdCard::Response::SharedPtr& res)
{
  if (camera_.formatSdCard()) {
    res->success = true;
    res->message.clear();
  }
  else {
    res->success = false;
    res->message = "The camera rejected the request to format the SD card.";
  }
}

void CxGb400PublisherNode::setPhotoQualityCb(
  const tobas_camera_msgs::srv::SetPhotoQuality::Request::ConstSharedPtr& req,
  const tobas_camera_msgs::srv::SetPhotoQuality::Response::SharedPtr& res)
{
  driver::CxGb400::PhotoQuality photo_quality;
  if (req->photo_quality == req->SUPER_FINE) {
    photo_quality = driver::CxGb400::PhotoQuality::kSuperFine;
  }
  else if (req->photo_quality == req->FINE) {
    photo_quality = driver::CxGb400::PhotoQuality::kFine;
  }
  else if (req->photo_quality == req->NORMAL) {
    photo_quality = driver::CxGb400::PhotoQuality::kNormal;
  }
  else {
    res->success = false;
    res->message = "Requested photo quality is out of range.";
    return;
  }
  if (camera_.setPhotoQuality(photo_quality)) {
    res->success = true;
    res->message.clear();
  }
  else {
    res->success = false;
    res->message = "The camera rejected the request to set the photo quality.";
  }
}

void CxGb400PublisherNode::setVideoFrameRateCb(
  const tobas_camera_msgs::srv::SetVideoFrameRate::Request::ConstSharedPtr& req,
  const tobas_camera_msgs::srv::SetVideoFrameRate::Response::SharedPtr& res)
{
  driver::CxGb400::VideoFrameRate video_frame_rate;
  if (req->video_frame_rate == req->K30FPS) {
    video_frame_rate = driver::CxGb400::VideoFrameRate::k30p;
  }
  else if (req->video_frame_rate == req->K60FPS) {
    video_frame_rate = driver::CxGb400::VideoFrameRate::k60p;
  }
  else {
    res->success = false;
    res->message = "Requested video frame rate is out of range.";
    return;
  }
  if (camera_.setVideoFrameRate(video_frame_rate)) {
    res->success = true;
    res->message.clear();
  }
  else {
    res->success = false;
    res->message = "The camera rejected the request to set the video frame rate.";
  }
}

void CxGb400PublisherNode::setVideoQualityCb(
  const tobas_camera_msgs::srv::SetVideoQuality::Request::ConstSharedPtr& req,
  const tobas_camera_msgs::srv::SetVideoQuality::Response::SharedPtr& res)
{
  driver::CxGb400::VideoQuality video_quality;
  if (req->video_quality == req->K4K) {
    video_quality = driver::CxGb400::VideoQuality::k4K;
  }
  else if (req->video_quality == req->K2_7K) {
    video_quality = driver::CxGb400::VideoQuality::k2_7K;
  }
  else if (req->video_quality == req->FHD) {
    video_quality = driver::CxGb400::VideoQuality::kFHD;
  }
  else if (req->video_quality == req->HD) {
    video_quality = driver::CxGb400::VideoQuality::kHD;
  }
  else {
    res->success = false;
    res->message = "Requested video quality is out of range.";
    return;
  }
  if (camera_.setVideoResolution(video_quality)) {
    res->success = true;
    res->message.clear();
  }
  else {
    res->success = false;
    res->message = "The camera rejected the request to set the video resolution.";
  }
}

void CxGb400PublisherNode::startRecordingCb(
  const tobas_camera_msgs::srv::StartRecording::Request::ConstSharedPtr&,
  const tobas_camera_msgs::srv::StartRecording::Response::SharedPtr& res)
{
  if (camera_.startRecording()) {
    res->success = true;
    res->message.clear();
  }
  else {
    res->success = false;
    res->message = "The camera rejected the request to start the recording.";
  }
}

void CxGb400PublisherNode::stopRecordingCb(
  const tobas_camera_msgs::srv::StopRecording::Request::ConstSharedPtr&,
  const tobas_camera_msgs::srv::StopRecording::Response::SharedPtr& res)
{
  if (camera_.stopRecording()) {
    res->success = true;
    res->message.clear();
  }
  else {
    res->success = false;
    res->message = "The camera rejected the request to stop the recording.";
  }
}

void CxGb400PublisherNode::takePictureToSdCb(
  const tobas_camera_msgs::srv::TakePictureToSd::Request::ConstSharedPtr&,
  const tobas_camera_msgs::srv::TakePictureToSd::Response::SharedPtr& res)
{
  if (camera_.takePictureToSd()) {
    res->success = true;
    res->message.clear();
  }
  else {
    res->success = false;
    res->message = "The camera rejected the request to take a picture and save it to the SD card.";
  }
}
}  // namespace camera
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::camera::CxGb400PublisherNode)
