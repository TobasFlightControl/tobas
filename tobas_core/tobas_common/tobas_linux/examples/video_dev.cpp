#include <fcntl.h>
#include <unistd.h>

#include <iostream>

#include "tobas_linux/video_dev.hpp"

int main(int argc, char** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <video Device>" << std::endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];  // e.g. /dev/video0

  linux::VideoDev camera;
  if (!camera.initialize(device, "MJPG")) {  // MJPG, JPEG, YUYV, etc.
    std::cerr << "Failed to initialize." << std::endl;
    return EXIT_FAILURE;
  }
  camera.displaySupportedFormats();
  if (!camera.startStream()) {
    std::cerr << "Failed to start stream." << std::endl;
    return EXIT_FAILURE;
  }
  if (!camera.takePicture()) {
    std::cerr << "Failed to take a picture." << std::endl;
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
  std::cout << "Successfully filmed out.jpg" << std::endl;

  return 0;
}
