#include <unistd.h>

#include <iostream>

#include <tobas_ic_drivers/stmicro/ism330dlc.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <SPI Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];

  stm::ISM330DLC imu;
  double ax, ay, az, gx, gy, gz;

  if (!imu.initialize(device)) {
    cerr << "Failed to initialize IMU." << endl;
    return EXIT_FAILURE;
  }

  if (!imu.setAccelOutputDataRate(stm::ISM330DLC::odr_xl_t::ODR_XL_6664HZ)) {
    cerr << "Failed to set accelerometer output data rate." << endl;
    return EXIT_FAILURE;
  }
  if (!imu.setGyroOutputDataRate(stm::ISM330DLC::odr_g_t::ODR_G_6664HZ)) {
    cerr << "Failed to set gyroscope output data rate." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!imu.readImu(ax, ay, az, gx, gy, gz)) {
      cerr << "Failed to read IMU." << endl;
      return EXIT_FAILURE;
    }

    cout << "Accel [m/s^2]: " << ax << ", " << ay << ", " << az << endl;
    cout << "Gyro [rad/s] : " << gx << ", " << gy << ", " << gz << endl;

    usleep(100000);
  }

  return EXIT_SUCCESS;
}
