#include <memory>
#include <string>

#include <navio2/Util.h>
#include <navio2/MPU9250.h>
#include <navio2/LSM9DS1.h>

std::unique_ptr<InertialSensor> get_inertial_sensor(std::string sensor_name)
{
  if (sensor_name == "mpu")
  {
    printf("Selected: MPU9250\n");
    auto ptr = std::unique_ptr<InertialSensor>{ new MPU9250() };
    return ptr;
  }
  else if (sensor_name == "lsm")
  {
    printf("Selected: LSM9DS1\n");
    auto ptr = std::unique_ptr<InertialSensor>{ new LSM9DS1() };
    return ptr;
  }
  else
  {
    return nullptr;
  }
}

void print_help()
{
  printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
  printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
}

std::string get_sensor_name(int argc, char* argv[])
{
  if (argc < 2)
  {
    printf("Enter parameter\n");
    print_help();
    return std::string();
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
        print_help();
        return "-1";
      case '?':
        printf("Wrong parameter.\n");
        print_help();
        return std::string();
    }
  }

  return std::string();
}
//=============================================================================
int main(int argc, char* argv[])
{
  if (check_apm())
  {
    return 1;
  }

  auto sensor_name = get_sensor_name(argc, argv);
  if (sensor_name.empty())
    return EXIT_FAILURE;

  auto sensor = get_inertial_sensor(sensor_name);

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

  while (1)
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
