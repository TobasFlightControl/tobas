#include <iostream>
#include <unistd.h>

#include <tobas_aso_core/ads1220.hpp>

using namespace std;

int main()
{
  aso::ADS1220 adc;
  double voltage;

  if (!adc.initialize())
  {
    cerr << "Failed to initialize ADC." << endl;
    return EXIT_FAILURE;
  }

  while (true)
  {
    if (!adc.readVoltage(voltage))
    {
      cerr << "Failed to read voltage channel." << endl;
      return EXIT_FAILURE;
    }

    cout << "Voltage Channel: " << voltage << endl;

    sleep(1);
  }

  return EXIT_SUCCESS;
}
