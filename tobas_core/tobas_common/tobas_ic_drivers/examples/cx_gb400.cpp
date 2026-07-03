// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <thread>

#include <tobas_ic_drivers/cx_gb400.hpp>

using namespace std::chrono_literals;
namespace ch = std::chrono;

int main(int argc, char** argv)
{
  if ((argc != 2) && (argc != 3)) {
    std::cerr << "Usage: " << argv[0] << " <video Device> --reset(optional)" << std::endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];  // e.g. /dev/video0
  bool reset_uavcan = false;
  if (argc == 3) {
    if (std::string(argv[2]) == "--reset") {
      reset_uavcan = true;
    }
  }

  tobas::driver::CxGb400 camera;
  // reset uavcan
  if (reset_uavcan) {
    if (!camera.initialize(device, tobas::driver::CxGb400::CameraPosition::kLower)) {
      std::cerr << "Failed to initialize cx_gb400." << std::endl;
    }
    if (!camera.fullReset()) {
      std::cerr << "Failed to full reset." << std::endl;
      return EXIT_FAILURE;
    }
    if (!camera.turnOffUavcan()) {
      std::cerr << "Failed to turn off UAVCAN." << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "WAIT for the camera's restart. After restart, turn off the camera." << std::endl;
    return EXIT_SUCCESS;
  }

  // usual operations without resetting uavcan.
  if (!camera.initialize(device, tobas::driver::CxGb400::CameraPosition::kLower)) {
    std::cerr << "Failed to initialize cx_gb400." << std::endl;
    return EXIT_FAILURE;
  }
  int cnt = 0;
  bool is_first_time = true;
  ch::system_clock::time_point last_send, now;
  last_send = ch::system_clock::now();
  now = last_send;
  while (true) {
    if (ch::duration_cast<ch::milliseconds>(now - last_send) > camera.kSendAttitudeInterval) {
      if (!camera.sendCopterAttitude(1.0, 0.0, 0.0, 0.0)) {
        std::cerr << "Failed to send attitude." << std::endl;
        return EXIT_FAILURE;
      }
      last_send = now;
    }
    if (is_first_time) {
      if (!camera.startStream()) {
        std::cerr << "Failed to start stream." << std::endl;
        return EXIT_FAILURE;
      }
      std::this_thread::sleep_for(50ms);
      if (!camera.takePicture()) {
        std::cerr << "Failed to take a picture." << std::endl;
        return EXIT_FAILURE;
      }
      if (!camera.sendGimbalCtrl(-30.0, 60.0)) {
        std::cerr << "Failed to send gimbal ctrl." << std::endl;
        return EXIT_FAILURE;
      }
      uint32_t image_size = 0;
      void* image_ptr = camera.getImage(image_size);
      // save image
      int out = open("out.jpg", O_RDWR | O_CREAT, S_IRWXU | S_IRWXO | S_IRWXG);
      if (out < 0) {
        std::cerr << "file error" << std::endl;
      }
      if (!write(out, image_ptr, image_size)) {
        std::cerr << "Failed to write image." << std::endl;
      }
      close(out);
      is_first_time = false;
    }
    std::this_thread::sleep_for(50ms);  // Can this be increased up to 30 Hz?
    now = ch::system_clock::now();
    if (++cnt > 20) {
      break;
    }
  }

  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
