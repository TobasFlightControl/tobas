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
 * @brief Video device driver. Controls UVC cameras using V4L2 (Video for Linux 2).
 * Supports taking pictures, capturing video, listing supported device formats, and sending UVC commands.
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
  virtual ~VideoDev();

  /* Initial setup: open the device, allocate memory, and so on. `pixcel_format` is four characters, such as MJPG, JPEG, or YUYV. */
  bool initialize(
    const char* video_dev,
    const char* pixcel_format = "MJPG",
    const bool& disable_video_streaming = false,
    const uint32_t& width = 0,
    const uint32_t& height = 0);

  /* Display supported image formats. */
  void displaySupportedFormats();

  /**
   * @brief Access Extension Unit Control directly.
   * ref: https://docs.kernel.org/userspace-api/media/drivers/uvcvideo.html#extension-unit-xu-support
   */
  bool execUvcControl(const uvc_xu_control_query& query);

  /* Turn the stream on so the PC can retrieve data from the device. */
  bool startStream();

  /* Dequeue and re-enqueue a buffer. Available only after the stream is on. */
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

  /* Check device capabilities. */
  bool checkCapability();

  /* Request buffers from the device. */
  bool requestDeviceBuffer();

  /* Allocate PC-side buffers. */
  bool mapBuffer();

  /* Fill one image worth of memory in the i-th device-side buffer. */
  bool enqueue(const uint32_t& i);

  /* Fill device-side buffers. */
  bool fillDeviceBuffer();

  /* Tell the device to allow streaming. */
  bool streamOn();

  /* Tell the device to stop streaming. */
  bool streamOff();

  /* Dequeue a buffer. Available only after the stream is on. Returns -1 on error. */
  int dequeue();

  /* Set the image format. */
  bool setImgFormat(const char* pixcel_format, const uint32_t& width = 0, const uint32_t& height = 0);

  /* Request the image format. */
  bool requestImgFormat();
};
}  // namespace linux
}  // namespace tobas
