#include <unistd.h>
#include <stdio.h>
#include <pthread.h>

#include <Common/MS5611.h>
#include <Common/Util.h>

void* acquireBarometerData(void* barom)
{
  MS5611* barometer = (MS5611*)barom;

  while (true)
  {
    barometer->refreshPressure();
    usleep(10000);  // Waiting for pressure data ready
    barometer->readPressure();

    barometer->refreshTemperature();
    usleep(10000);  // Waiting for temperature data ready
    barometer->readTemperature();

    barometer->calculatePressureAndTemperature();

    // sleep(0.5);
  }

  pthread_exit(nullptr);
}

int main()
{
  if (check_apm())
  {
    return 1;
  }

  MS5611 baro;
  baro.initialize();

  pthread_t baro_thread;

  if (pthread_create(&baro_thread, nullptr, acquireBarometerData, (void*)&baro))
  {
    printf("Error: Failed to create barometer thread\n");
    return 0;
  }

  while (true)
  {
    printf(
      "Temperature(C): %f Pressure(millibar): %f\n", baro.getTemperature(), baro.getPressure());
    sleep(1);
  }

  pthread_exit(nullptr);

  return 1;
}
