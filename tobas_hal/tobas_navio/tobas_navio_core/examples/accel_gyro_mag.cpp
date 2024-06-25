#include <memory>
#include <string>
#include <unistd.h>

#include <tobas_navio_core/util.hpp>
#include <tobas_navio_core/mpu9250.hpp>
#include <tobas_navio_core/lsm9ds1.hpp>

using namespace std;
using namespace navio;

unique_ptr<InertialSensor> getInertialSensor(const string& sensor_name)
{
  if (sensor_name == "mpu")
  {
    printf("Selected: MPU9250\n");
    auto ptr = unique_ptr<InertialSensor>{ new MPU9250() };
    return ptr;
  }
  else if (sensor_name == "lsm")
  {
    printf("Selected: LSM9DS1\n");
    auto ptr = unique_ptr<InertialSensor>{ new LSM9DS1() };
    return ptr;
  }
  else
  {
    return nullptr;
  }
}

void printHelp()
{
  printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
  printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
}

string getSensorName(int argc, char* argv[])
{
  if (argc < 2)
  {
    printf("Enter parameter\n");
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
        printf("Wrong parameter.\n");
        printHelp();
        return "";
    }
  }

  return "";
}

int main(int argc, char* argv[])
{
  if (checkAPM())
    return 1;

  const auto sensor_name = getSensorName(argc, argv);
  if (sensor_name.empty())
    return EXIT_FAILURE;

  const auto sensor = getInertialSensor(sensor_name);

  if (!sensor)
  {
    printf("Wrong sensor name. Select: mpu or lsm\n");
    return EXIT_FAILURE;
  }

  if (!sensor->probe())
  {
    printf("Sensor not enabled\n");
    return EXIT_FAILURE;
  }
  sensor->initialize();

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  //-------------------------------------------------------------------------

  while (true)
  {
    sensor->update();
    sensor->readAccelerometer(&ax, &ay, &az);
    sensor->readGyroscope(&gx, &gy, &gz);
    sensor->readMagnetometer(&mx, &my, &mz);
    printf("Acc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
    printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
    printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);

    // usleep(500000);
    usleep(5000);
  }
}
