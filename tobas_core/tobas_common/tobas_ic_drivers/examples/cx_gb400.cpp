#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <thread>

#include <tobas_ic_drivers/cx_gb400.hpp>

// #define UAVCAN_RESET // uncomment this if you would like to turn off UAVCAN

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <video Device>" << std::endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];  // e.g. /dev/video0

  driver::CxGb400 camera;
  if (!camera.initialize(device, camera.LOWER)) {
    std::cerr << "Failed to initialize cx_gb400." << std::endl;
#ifndef UAVCAN_RESET
    return EXIT_FAILURE;
#endif
  }
#ifdef UAVCAN_RESET
  if (!camera.fullReset()) {
    std::cerr << "Failed to full reset." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "here2" << std::endl;
  if (!camera.turnOffUAVCAN()) {
    std::cerr << "Failed to turn off UAVCAN." << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "WAIT for the camera's restart. After restart, turn off the camera." << std::endl;
#else
  int cnt = 0;
  bool is_first_time = true;
  std::chrono::system_clock::time_point last_send, now;
  last_send = std::chrono::system_clock::now();
  now = last_send;
  while (true) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send) > camera.att_send_interval) {
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
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (!camera.takePicture()) {
        std::cerr << "Failed to take a picture." << std::endl;
        return EXIT_FAILURE;
      }
      if (!camera.sendGimbalCtrl(-30.0, 60.0)) {
        std::cerr << "Failed to send gimbal ctrl." << std::endl;
        return EXIT_FAILURE;
      }
      uint image_size = 0;
      void* image_ptr = camera.getImage(image_size);
      // save image
      int out = open("out.jpg", O_RDWR | O_CREAT, S_IRWXU | S_IRWXO | S_IRWXG);
      if (out == -1) {
        std::cerr << "file error" << std::endl;
      }
      if (!write(out, image_ptr, image_size)) {
        std::cerr << "Failed to write image." << std::endl;
      }
      close(out);
      is_first_time = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 30Hzまであげられる？
    now = std::chrono::system_clock::now();
    cnt++;
    if (cnt > 20) {
      break;
    }
  }
#endif
  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
