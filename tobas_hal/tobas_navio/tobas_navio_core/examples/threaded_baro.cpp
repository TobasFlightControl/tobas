#include <iostream>
#include <unistd.h>
#include <pthread.h>

#include <tobas_navio_core/ms5611.hpp>
#include <tobas_navio_core/util.hpp>

using namespace std;
using namespace navio;

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

    sleep(0.5);
  }

  pthread_exit(nullptr);
}

int main()
{
  if (checkAPM())
    return 1;

  MS5611 baro;
  baro.initialize();

  pthread_t baro_thread;

  if (pthread_create(&baro_thread, nullptr, acquireBarometerData, (void*)&baro))
  {
    cout << "Error: Failed to create barometer thread" << endl;
    return 0;
  }

  while (true)
  {
    cout << "Temperature[C]: " << baro.getTemperature() << ", Pressure[Pa]: " << baro.getPressure() << endl;
    sleep(1);
  }

  pthread_exit(nullptr);

  return 1;
}
