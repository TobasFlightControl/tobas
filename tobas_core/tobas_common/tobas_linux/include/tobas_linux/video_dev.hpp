// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <linux/usb/video.h>
#include <linux/uvcvideo.h>

#include <cstdint>
#include <string>

namespace tobas
{
namespace linux
{
/**
 * @brief video deviceドライバ．v4l2 (video for linux 2)を用いてuvcカメラの制御を行う．
 * 写真撮影，映像の取得，デバイスが対応している形式の取得，UVC commandの送信などが可能．
 */
class VideoDev
{
  static constexpr uint32_t kBufferSize = 3;

public:
  struct ImgFormat
  {
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t bytes_per_line;
  };

  explicit VideoDev();
  ~VideoDev();

  /* 初期動作: device open, memory確保など．pixcel_formatは4文字で (MJPG, JPEG, YUYV, etc.)． */
  bool initialize(
    const char* video_dev,
    const char* pixcel_format = "MJPG",
    const bool& disable_video_streaming = false,
    const uint32_t& width = 0,
    const uint32_t& height = 0);

  /* サポートしている画像のフォーマットを表示する． */
  void displaySupportedFormats();

  /**
   * @brief Access Extension Unit Control directly.
   * ref: https://docs.kernel.org/userspace-api/media/drivers/uvcvideo.html#extension-unit-xu-support
   */
  bool execUvcControl(const uvc_xu_control_query& query);

  /* streamをONにしてPCからdeviceのデータを取り出せるようにする． */
  bool startStream();

  /* dequeueして，その分のデータをenqueueする．streamをONにしてからでないと使用不可． */
  bool takePicture();

  void* getImage(uint32_t& length);

  ImgFormat getImageFormat();

  std::string FCC2S(const uint32_t& val);

private:
  int fd_ = -1;
  int image_address_ = -1;
  bool buffer_mapped_ = false;
  ImgFormat fmt_;

  struct Buffer
  {
    void* start = nullptr;
    size_t length = 0;
  };
  Buffer* buffers_;

  bool is_stream_on_ = false;

  /* deviceのcapabilityを確認． */
  bool checkCapability();

  /* deviceにbufferを要求する． */
  bool requestDeviceBuffer();

  /* PC側buffer確保． */
  bool mapBuffer();

  /* device側のbufferのi番目の画像1つ分のメモリを埋める． */
  bool enqueue(const uint32_t& i);

  /* device側bufferを埋める． */
  bool fillDeviceBuffer();

  /* deviceにstreamを許可するように通達． */
  bool streamOn();

  /* deviceにstreamを止めるように通達． */
  bool streamOff();

  /* dequeueする．streamをONにしてからでないと使用不可．errorのときは-1を返す． */
  int dequeue();

  /* 画像のフォーマットを指定する． */
  bool setImgFormat(const char* pixcel_format, const uint32_t& width = 0, const uint32_t& height = 0);

  /* 画像のフォーマットを取得する． */
  bool requestImgFormat();
};
}  // namespace linux
}  // namespace tobas
