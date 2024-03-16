#include <unistd.h>
#include <stdio.h>

#include <tobas_navio_core/ms5611.hpp>
#include <tobas_navio_core/util.hpp>

using namespace navio;

int main()
{
  MS5611 barometer;

  if (check_apm())
  {
    return 1;
  }

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

    printf(
      "Temperature[C]: %f Pressure[Pa]: %f\n", barometer.getTemperature(), barometer.getPressure());

    sleep(1);
  }

  return 0;
}
