// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/time.h>
#include <libswscale/swscale.h>

#include <tobas_node/node.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace tobas
{
namespace camera
{
class FFmpegToROSMsgConverter : public BaseNode
{
public:
  explicit FFmpegToROSMsgConverter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~FFmpegToROSMsgConverter();

private:
  bool initialize();
  void timerCallback();
  bool convertFrameToMessage(const AVFrame* frame, const sensor_msgs::msg::Image::UniquePtr& image);

  std::string output_msg_encoding_;
  std::string frame_id_;

  bool initialized_ = false;
  std::string input_url_;
  AVPixelFormat av_pix_fmt_;

  ros2::PublisherPtr<sensor_msgs::msg::Image> img_pub_;
  ros2::TimerPtr timer_;

  AVFormatContext* format_context_ = nullptr;
  AVStream* video_stream_ = nullptr;
  AVCodecContext* codec_context_ = nullptr;
  SwsContext* sws_context_ = nullptr;
  AVFrame* output_frame_ = nullptr;
};

FFmpegToROSMsgConverter::FFmpegToROSMsgConverter(const rclcpp::NodeOptions& options)
  : BaseNode("ffmpeg_to_ros_msg_converter", nodeOptions_Default(options))
{
  const auto ros_image_topic_name = getStringParam("ros_image_topic", "image");
  // ffmpegが送信してくるデータのプロトコルの名称．udp, srtなど．
  const auto protocol = getStringParam("protocol", "srt");
  output_msg_encoding_ = getStringParam("output_msg_encoding", "rgb8");
  frame_id_ = getStringParam("frame_id", "map");
  const auto fps = getIntParam("FPS", 30);  // ffmpegが送信してくる映像データのfpsより高い値であればok．
  // ffmpegが送信してくるデータの受信側ipアドレスとport番号．
  const auto port_uri = getStringParam("port_uri", "127.0.0.1:8888");

  input_url_ = protocol + "://" + port_uri + "?mode=listener&listen_timeout=5000000";

  // Set AV pixel format
  // ref: https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder/blob/master/src/utils.cpp
  const std::unordered_map<std::string, AVPixelFormat> ros_to_av_pix_map = {
    { "bayer_rggb8", AV_PIX_FMT_BAYER_RGGB8 },
    { "bayer_bggr8", AV_PIX_FMT_BAYER_BGGR8 },
    { "bayer_gbrg8", AV_PIX_FMT_BAYER_GBRG8 },
    { "bayer_grbg8", AV_PIX_FMT_BAYER_GRBG8 },
    { "bayer_rggb16", AV_PIX_FMT_BAYER_RGGB16LE },  // map to little endian :(
    { "bayer_bggr16", AV_PIX_FMT_BAYER_BGGR16LE },
    { "bayer_gbrg16", AV_PIX_FMT_BAYER_GBRG16LE },
    { "bayer_grbg16", AV_PIX_FMT_BAYER_GRBG16LE },
    { "rgb8", AV_PIX_FMT_RGB24 },
    { "rgba8", AV_PIX_FMT_RGBA },
    { "rgb16", AV_PIX_FMT_RGB48LE },
    { "rgba16", AV_PIX_FMT_RGBA64LE },
    { "bgr8", AV_PIX_FMT_BGR24 },
    { "bgra8", AV_PIX_FMT_BGRA },
    { "bgr16", AV_PIX_FMT_BGR48LE },
    { "bgra16", AV_PIX_FMT_BGRA64LE },
    { "mono8", AV_PIX_FMT_GRAY8 },
    { "mono16", AV_PIX_FMT_GRAY16LE },
    { "yuv422", AV_PIX_FMT_YUV422P },           // deprecated, not sure correct
    { "uyvy", AV_PIX_FMT_UYVY422 },             // not sure that is correct
    { "yuyv", AV_PIX_FMT_YUYV422 },             // not sure that is correct
    { "yuv422_yuy2", AV_PIX_FMT_YUV422P16LE },  // deprecated, probably wrong
    { "nv21", AV_PIX_FMT_NV21 },
    { "nv24", AV_PIX_FMT_NV24 },
    { "nv12", AV_PIX_FMT_NV12 }  // not an official ROS encoding!!
  };
  const auto av_pix_map_it = ros_to_av_pix_map.find(output_msg_encoding_);
  if (av_pix_map_it == ros_to_av_pix_map.end()) {
    TOBAS_ERROR("Encoding ", output_msg_encoding_, " not supported.");
    return;
  }
  av_pix_fmt_ = av_pix_map_it->second;

  img_pub_ = createPublisher<sensor_msgs::msg::Image>(ros_image_topic_name);
  timer_ = createTimer(std::chrono::milliseconds(1000 / fps), &FFmpegToROSMsgConverter::timerCallback, this);
}

FFmpegToROSMsgConverter::~FFmpegToROSMsgConverter()
{
  av_frame_free(&output_frame_);
}

bool FFmpegToROSMsgConverter::initialize()
{
  format_context_ = avformat_alloc_context();
  if (avformat_open_input(&format_context_, input_url_.c_str(), nullptr, nullptr) != 0) {
    TOBAS_ERROR("Listen timeout. Check connection.");
    avformat_close_input(&format_context_);
    return false;
  }

  // get stream info
  if (avformat_find_stream_info(format_context_, nullptr) < 0) {
    TOBAS_ERROR("avformat_find_stream_info failed");
    return false;
  }

  // find video stream
  for (uint32_t i = 0; i < format_context_->nb_streams; ++i) {
    if (format_context_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
      video_stream_ = format_context_->streams[i];
      break;
    }
  }
  if (!video_stream_) {
    TOBAS_ERROR("no video stream found");
    return false;
  }

  // decoder作成
  // find decoder
  const AVCodec* codec = avcodec_find_decoder(video_stream_->codecpar->codec_id);
  if (!codec) {
    TOBAS_ERROR("No supported decoder");
    return false;
  }

  // alloc codec context
  codec_context_ = avcodec_alloc_context3(codec);
  if (!codec_context_) {
    TOBAS_ERROR("Failed to allocate codec context");
    return false;
  }

  // open codec
  if (avcodec_parameters_to_context(codec_context_, video_stream_->codecpar) < 0) {
    TOBAS_ERROR("avcodec_parameters_to_context failed");
    return false;
  }
  if (avcodec_open2(codec_context_, codec, nullptr) < 0) {
    TOBAS_ERROR("avcodec_open2 failed");
    return false;
  }

  // allocate frame memory for message data
  output_frame_ = av_frame_alloc();

  return true;
}

void FFmpegToROSMsgConverter::timerCallback()
{
  // initialize if not initialized
  if (!initialized_) {
    if (!initialize()) {
      TOBAS_ERROR("Failed to initialize");
      return;
    }
    else {
      initialized_ = true;
    }
  }

  // decode frames
  auto frame = av_frame_alloc();
  AVPacket packet;

  // frameが1個読み込まれる
  if (av_read_frame(format_context_, &packet) < 0) {
    return;
  }
  if (packet.stream_index == video_stream_->index) {
    // decoderにpacketを送りつける
    if (avcodec_send_packet(codec_context_, &packet) != 0) {
      TOBAS_ERROR("avcodec_send_packet failed");
    }

    // decodeしてもらったデータをframeに格納する
    while (avcodec_receive_frame(codec_context_, frame) == 0) {
      auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
      image_msg->height = frame->height;
      image_msg->width = frame->width;
      image_msg->step = (sensor_msgs::image_encodings::bitDepth(output_msg_encoding_) / 8) * image_msg->width *
                        sensor_msgs::image_encodings::numChannels(output_msg_encoding_);
      image_msg->encoding = output_msg_encoding_;
      if (!convertFrameToMessage(frame, image_msg)) {
        TOBAS_ERROR("Failed to convert frame to message.");
      }
      image_msg->header.frame_id = frame_id_;
      image_msg->header.stamp = this->get_clock()->now();
      img_pub_->publish(std::move(image_msg));
    }
  }

  // av_read_frame()をやったら必ずこれを呼んでメモリを開放しないといけない
  av_packet_unref(&packet);
}

bool FFmpegToROSMsgConverter::convertFrameToMessage(const AVFrame* frame, const sensor_msgs::msg::Image::UniquePtr& image)
{
  if (!sws_context_) {  // initialize
    output_frame_->format = av_pix_fmt_;
    sws_context_ = sws_getContext(
      frame->width,
      frame->height,
      static_cast<AVPixelFormat>(frame->format),  // src
      frame->width,
      frame->height,
      av_pix_fmt_,  // dest
      SWS_FAST_BILINEAR | SWS_ACCURATE_RND,
      nullptr,
      nullptr,
      nullptr);
    if (!sws_context_) {
      TOBAS_ERROR("Failed to allocate sws_context.");
      return false;
    }
  }

  const auto buffer_size = av_image_get_buffer_size(
    static_cast<AVPixelFormat>(output_frame_->format),
    frame->width,
    frame->height,
    1  // alignment
  );
  image->data.resize(buffer_size);

  if (
    av_image_fill_arrays(
      output_frame_->data,
      output_frame_->linesize,
      &(image->data[0]),
      static_cast<AVPixelFormat>(output_frame_->format),
      frame->width,
      frame->height,
      1) < 0) {
    TOBAS_ERROR("av_image_fill_arrays failed.");
    return false;
  }

  sws_scale(
    sws_context_,
    frame->data,
    frame->linesize,
    0,  // src
    frame->height,
    output_frame_->data,
    output_frame_->linesize);  // dest

  return true;
}
}  // namespace camera
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::camera::FFmpegToROSMsgConverter)
