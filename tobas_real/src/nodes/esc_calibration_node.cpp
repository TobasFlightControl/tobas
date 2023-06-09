#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_real/esc_calibration.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "esc_calibration");

  const int error_code = tobas_real::calibrateEscs();
  if (error_code == 0)
  {
    rosInfo("Calibration finished successfully.");
  }
  else
  {
    rosError("Calibration failed.");
  }
}
