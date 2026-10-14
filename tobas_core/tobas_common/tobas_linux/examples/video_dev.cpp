// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <fcntl.h>
#include <unistd.h>

#include <iostream>

#include "tobas_linux/video_dev.hpp"

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <video Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];  // e.g. /dev/video0

  tobas::linux::VideoDev camera;
  if (!camera.initialize(device, "MJPG")) {  // MJPG, JPEG, YUYV, etc.
    std::cerr << "Failed to initialize.\n";
    return EXIT_FAILURE;
  }
  camera.displaySupportedFormats();
  if (!camera.startStream()) {
    std::cerr << "Failed to start stream.\n";
    return EXIT_FAILURE;
  }
  if (!camera.takePicture()) {
    std::cerr << "Failed to take a picture.\n";
    return EXIT_FAILURE;
  }
  uint32_t image_size = 0;
  void* image_ptr = camera.getImage(image_size);
  // Save image.
  int out = open("out.jpg", O_RDWR | O_CREAT, S_IRWXU | S_IRWXO | S_IRWXG);
  if (out < 0) {
    std::cerr << "file error\n";
  }
  if (!write(out, image_ptr, image_size)) {
    std::cerr << "Failed to write image.\n";
  }
  close(out);
  std::cout << "Successfully filmed out.jpg\n";

  return 0;
}
