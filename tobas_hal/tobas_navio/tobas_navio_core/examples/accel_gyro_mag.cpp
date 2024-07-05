#include <memory>
#include <string>
#include <iostream>
#include <unistd.h>

#include <tobas_navio_core/util.hpp>
#include <tobas_navio_core/mpu9250.hpp>
#include <tobas_navio_core/lsm9ds1.hpp>

using namespace std;

unique_ptr<navio::InertialSensor> getInertialSensor(const string& sensor_name)
{
  if (sensor_name == "mpu")
  {
    cout << "Selected: MPU9250" << endl;
    auto ptr = unique_ptr<navio::InertialSensor>{ new navio::MPU9250() };
    return ptr;
  }
  else if (sensor_name == "lsm")
  {
    cout << "Selected: LSM9DS1" << endl;
    auto ptr = unique_ptr<navio::InertialSensor>{ new navio::LSM9DS1() };
    return ptr;
  }
  else
  {
    return nullptr;
  }
}

void printHelp()
{
  cout << "Possible parameters:\nSensor selection: -i [sensor name]" << endl;
  cout << "Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h" << endl;
}

string getSensorName(int argc, char* argv[])
{
  if (argc < 2)
  {
    cout << "Enter parameter" << endl;
    printHelp();
    return "";
  }

  // prevent the error message
  opterr = 0;
  int parameter;

  while ((parameter = getopt(argc, argv, "i:h")) != -1)
  {
    switch (parameter)
    {
      case 'i':
        return optarg;
      case 'h':
        printHelp();
        return "-1";
      case '?':
        cerr << "Wrong parameter." << endl;
        printHelp();
        return "";
    }
  }

  return "";
}

int main(int argc, char* argv[])
{
  if (navio::checkAPM())
    return EXIT_FAILURE;

  const auto sensor_name = getSensorName(argc, argv);
  if (sensor_name.empty())
    return EXIT_FAILURE;

  const auto sensor = getInertialSensor(sensor_name);

  if (!sensor)
  {
    cerr << "Wrong sensor name. Select: mpu or lsm" << endl;
    return EXIT_FAILURE;
  }

  if (!sensor->initialize())
  {
    cerr << "Failed to initialize IMU." << endl;
    return EXIT_FAILURE;
  }

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  while (true)
  {
    sensor->update();
    sensor->readAccelerometer(&ax, &ay, &az);
    sensor->readGyroscope(&gx, &gy, &gz);
    sensor->readMagnetometer(&mx, &my, &mz);

    cout << "Acc: " << ax << " " << ay << " " << az << endl;
    cout << "Gyr: " << gx << " " << gy << " " << gz << endl;
    cout << "Mag: " << mx << " " << my << " " << mz << endl;

    // usleep(500000);
    usleep(5000);
  }

  return EXIT_SUCCESS;
}
