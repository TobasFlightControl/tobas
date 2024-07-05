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
    return EXIT_FAILURE;

  if (!barometer.initialize())
  {
    cerr << "Failed to initialize barometer." << endl;
    return EXIT_FAILURE;
  }

  while (true)
  {
    if (!barometer.update())
    {
      cerr << "Failed to update barometer." << endl;
      continue;
    }

    const auto temp = barometer.getTemperature();
    const auto pres = barometer.getPressure();
    cout << "Temperature[C]: " << temp << ", Pressure[Pa]: " << pres << endl;

    sleep(1);
  }

  return EXIT_SUCCESS;
}
