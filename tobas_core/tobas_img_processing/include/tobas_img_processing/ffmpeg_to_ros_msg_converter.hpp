#pragma once

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include <tobas_node/node.hpp>

#include <sensor_msgs/msg/image.hpp>

class FFmpegToROSMsgConverter : public tobas::BaseNode
{
public:
  explicit FFmpegToROSMsgConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~FFmpegToROSMsgConverter();

private:
  bool initialize();
  void timerCallback();
  bool convertFrameToMessage(const AVFrame* frame, const sensor_msgs::msg::Image::UniquePtr& image);
  enum AVPixelFormat rosToAvPixFormat(const std::string & ros_pix_fmt);

  bool initialized_ = false;
  std::string input_url_;
  std::string output_msg_encoding_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  AVFormatContext* format_context_ = nullptr;
  AVStream* video_stream_ = nullptr;
  AVCodecContext* codec_context_ = nullptr;
  SwsContext* sws_context_ = nullptr;
  AVFrame* output_frame_ = nullptr;
};
