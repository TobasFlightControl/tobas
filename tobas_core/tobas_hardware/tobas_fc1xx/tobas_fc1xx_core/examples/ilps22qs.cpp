#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/ilps22qs.hpp"

using namespace std;

int main()
{
  fc1xx::ILPS22QS baro;
  double pres, temp;

  if (!baro.initialize()) {
    cerr << "Failed to initialize barometer." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!baro.readPressure(pres)) {
      cerr << "Failed to read pressure." << endl;
      continue;
    }

    if (!baro.readTemperature(temp)) {
      cerr << "Failed to read temperature." << endl;
      continue;
    }

    cout << "Pressure [hPa]     : " << pres / 100. << endl;
    cout << "Temperature [degC]: " << temp << endl;

    this_thread::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
