#include <iostream>
#include <unistd.h>

#include <tobas_t1_core/ilps22qs.hpp>

using namespace std;

int main()
{
  t1::ILPS22QS barometer;
  double pressure, temperature;

  if (!barometer.initialize()) {
    cerr << "Failed to initialize barometer." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!barometer.readPressure(pressure)) {
      cerr << "Failed to read pressure." << endl;
      return EXIT_FAILURE;
    }

    if (!barometer.readTemperature(temperature)) {
      cerr << "Failed to read temperature." << endl;
      return EXIT_FAILURE;
    }

    cout << "Pressure [Pa]     : " << pressure << endl;
    cout << "Temperature [degC]: " << temperature << endl;

    sleep(1);
  }

  return EXIT_SUCCESS;
}
