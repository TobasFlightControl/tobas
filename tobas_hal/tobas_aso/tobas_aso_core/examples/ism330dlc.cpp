#include <iostream>
#include <unistd.h>

#include <tobas_aso_core/ism330dlc.hpp>

using namespace std;

int main()
{
  aso::ISM330DLC imu;
  double ax, ay, az;
  double gx, gy, gz;

  if (!imu.initialize())
  {
    cerr << "Failed to initialize IMU." << endl;
    return EXIT_FAILURE;
  }

  while (true)
  {
    if (!imu.readAcc(ax, ay, az))
    {
      cerr << "Failed to read accel." << endl;
      return EXIT_FAILURE;
    }

    if (!imu.readGyro(gx, gy, gz))
    {
      cerr << "Failed to read gyro." << endl;
      return EXIT_FAILURE;
    }

    cout << "Accel [m/s^2]: " << ax << ", " << ay << ", " << az << endl;
    cout << "Gyro [rad/s] : " << gx << ", " << gy << ", " << gz << endl;

    sleep(1);
  }

  return EXIT_SUCCESS;
}
