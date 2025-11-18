#include "tobas_linux/video_dev.hpp"

#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <iostream>

namespace linux
{
VideoDev::VideoDev()
{
}

VideoDev::~VideoDev()
{
  if (is_stream_on) {
    streamOff();
  }

  if (buffer_mapped_) {
    for (uint i = 0; i < kBufferSize; ++i) {
      if (-1 == munmap(buffers[i].start, buffers[i].length)) {
        std::cerr << "Failed to munmap memory" << std::endl;
      }
    }
    free(buffers);
  }

  if (fd_ > 0) {
    close(fd_);
  }
}

bool VideoDev::initialize(const char* video_dev, const char* pixel_format, const bool& disable_video_streaming, const uint& width, const uint& height)
{
  if (fd_ >= 0) {
    close(fd_);
  }

  // open device
  fd_ = open(video_dev, O_RDWR);
  if (fd_ == -1) {
    std::cerr << "Failed to open video device: " << video_dev << std::endl;
    return false;
  }

  if (!checkCapability()) {
    return false;
  }

  if (!disable_video_streaming){
    if (!setImgFormat(pixel_format, width, height)) {
      return false;
    }

    if (!requestDeviceBuffer()) {
      return false;
    }

    if (!mapBuffer()) {
      return false;
    }

    if (!requestImgFormat()) {
      return false;
    }
  }
  return true;
}

void VideoDev::displaySupportedFormats()
{
  std::cout << "Supported Formats" << std::endl;
  struct v4l2_fmtdesc* current_format = new v4l2_fmtdesc();
  struct v4l2_frmsizeenum* current_size = new v4l2_frmsizeenum();
  struct v4l2_frmivalenum* current_interval = new v4l2_frmivalenum();

  current_format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  current_format->index = 0;
  for (current_format->index = 0; ioctl(fd_, VIDIOC_ENUM_FMT, current_format) == 0; ++current_format->index) {
    current_size->index = 0;
    current_size->pixel_format = current_format->pixelformat;

    for (current_size->index = 0; ioctl(fd_, VIDIOC_ENUM_FRAMESIZES, current_size) == 0; ++current_size->index) {
      current_interval->index = 0;
      current_interval->pixel_format = current_size->pixel_format;
      current_interval->width = current_size->discrete.width;
      current_interval->height = current_size->discrete.height;
      for (current_interval->index = 0; ioctl(fd_, VIDIOC_ENUM_FRAMEINTERVALS, current_interval) == 0;
           ++current_interval->index) {
        if (current_interval->type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          std::cout << "      Format : " << FCC2S(current_format->pixelformat)
                    << ", Size: " << current_size->discrete.width << " x " << current_size->discrete.height
                    << std::endl;
        }
      }  // interval loop
    }  // size loop
  }
}

bool VideoDev::execUvcControl(const uvc_xu_control_query& query)
{
  if (ioctl(fd_, UVCIOC_CTRL_QUERY, &query) == -1) {
    std::cerr << "Failed to execute UVC control. errno=" << errno << " : " << strerror(errno) << std::endl;
    return false;
  }
  return true;
}

bool VideoDev::startStream()
{
  if (!fillDeviceBuffer()) {
    return false;
  }
  if (!streamOn()) {
    return false;
  }
  is_stream_on = true;
  return true;
}

bool VideoDev::takePicture()
{
  image_address_ = dequeue();
  if (image_address_ < 0) {
    std::cerr << "Failed to dequeue." << std::endl;
    return false;
  }
  if (!enqueue(image_address_)) {
    return false;
  }
  return true;
}

void* VideoDev::getImage(uint& length)
{
  length = buffers[image_address_].length;
  return buffers[image_address_].start;
}

VideoDev::ImgFormat VideoDev::getImageFormat()
{
  return fmt_;
}

std::string VideoDev::FCC2S(const unsigned int& val)
{
  std::string s;
  s += val & 0x7f;
  s += (val >> 8) & 0x7f;
  s += (val >> 16) & 0x7f;
  s += (val >> 24) & 0x7f;
  if (val & (1 << 31)) {
    s += "-BE";
  }
  return s;
}

bool VideoDev::checkCapability()
{
  struct v4l2_capability cap = {};
  if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) {
    std::cerr << "Failed to query capability." << std::endl;
    return false;
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    std::cerr << "No vide capture capability." << std::endl;
    return false;
  }
  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    std::cerr << "No stream support" << std::endl;
    return false;
  }
  return true;
}

bool VideoDev::requestDeviceBuffer()
{
  struct v4l2_requestbuffers req = {};
  req.count = kBufferSize;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
    std::cerr << "Failed to request buffer." << std::endl;
    return false;
  }
  // 確保できた枚数の確認
  if (req.count < kBufferSize) {
    std::cerr << "Insufficient buffer memory on device." << std::endl;
    return false;
  }
  return true;
}

bool VideoDev::mapBuffer()
{
  buffers = static_cast<struct buffer*>(calloc(kBufferSize, sizeof(*buffers)));
  if (buffers == nullptr) {
    std::cerr << "Calloc failed." << std::endl;
    return false;
  }
  for (uint i = 0; i < kBufferSize; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
      std::cerr << "Failed to query buffer." << std::endl;
      return false;
    }
    buffers[i].length = buf.length;
    buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
  }
  buffer_mapped_ = true;
  return true;
}

bool VideoDev::enqueue(const uint i)
{
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = i;
  if (-1 == ioctl(fd_, VIDIOC_QBUF, &buf)) {
    std::cerr << "Failed to query buffer." << std::endl;
    return false;
  }
  return true;
}

bool VideoDev::fillDeviceBuffer()
{
  for (uint i = 0; i < kBufferSize; ++i) {
    enqueue(i);
  }
  return true;
}

bool VideoDev::streamOn()
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
    std::cerr << "Stream ON failed." << std::endl;
    return false;
  }
  return true;
}

bool VideoDev::streamOff()
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMOFF, &type) == -1) {
    std::cerr << "Stream Off failed." << std::endl;
    return false;
  }
  is_stream_on = false;
  return true;
}

int VideoDev::dequeue()
{
  if (!is_stream_on) {
    std::cerr << "Start stream before getting image." << std::endl;
    return -1;
  }
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
    std::cerr << "Failed to retrieve Frame" << std::endl;
    return -1;
  }
  return buf.index;
}

bool VideoDev::setImgFormat(const char* pixel_format, const uint& width, const uint& height)
{
  struct v4l2_format fmt_request = {};
  fmt_request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_G_FMT, &fmt_request) == -1) {
    std::cerr << "Failed to get image format." << std::endl;
    return false;
  }
  fmt_request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt_request.fmt.pix.pixelformat = v4l2_fourcc(
    pixel_format[0],
    pixel_format[1],
    pixel_format[2],
    pixel_format[3]);  // 本来はV4L2_PIX_FMT_MJPET, V4L2_PIX_FMT_YUYVなどと指定する
  fmt_request.fmt.pix.field = V4L2_FIELD_ANY;
  if ((width != 0) && (height != 0)){
    fmt_request.fmt.pix.width = width;
    fmt_request.fmt.pix.height = height;
  }
  if (ioctl(fd_, VIDIOC_S_FMT, &fmt_request) == -1) {
    std::cerr << "Failed to set image format. errno : " << errno << " : " << strerror(errno) << std::endl;
    return false;
  }
  // check format
  if (ioctl(fd_, VIDIOC_G_FMT, &fmt_request) == -1) {
    std::cerr << "Failed to get image format." << std::endl;
    return false;
  }
  if (fmt_request.fmt.pix.pixelformat != v4l2_fourcc(pixel_format[0], pixel_format[1], pixel_format[2], pixel_format[3])) {
    std::cerr << "Failed to set image format. The specified format is not supported by the device." << std::endl;
    return false;
  }
  return true;
}

bool VideoDev::requestImgFormat()
{
  struct v4l2_format fmt_request = {};
  fmt_request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_G_FMT, &fmt_request) == -1) {
    std::cerr << "Failed to get image format." << std::endl;
    return false;
  }
  fmt_.pixel_format = fmt_request.fmt.pix.pixelformat;
  fmt_.width = fmt_request.fmt.pix.width;
  fmt_.height = fmt_request.fmt.pix.height;
  fmt_.bytes_per_line = fmt_request.fmt.pix.bytesperline;
  return true;
}
}  // end of namespace linux
