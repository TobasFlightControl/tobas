#include <iostream>
#include <unistd.h>

#include <tobas_navio_core/ms5611.hpp>
#include <tobas_navio_core/util.hpp>

using namespace std;
using namespace navio;

int main()
{
  MS5611 barometer;

  if (checkAPM())
    return 1;

  barometer.initialize();

  while (true)
  {
    barometer.refreshPressure();
    usleep(10000);  // Waiting for pressure data ready
    barometer.readPressure();

    barometer.refreshTemperature();
    usleep(10000);  // Waiting for temperature data ready
    barometer.readTemperature();

    barometer.calculatePressureAndTemperature();

    cout << "Temperature[C]: " << barometer.getTemperature()
         << ", Pressure[Pa]: " << barometer.getPressure() << endl;

    sleep(1);
  }

  return 0;
}
