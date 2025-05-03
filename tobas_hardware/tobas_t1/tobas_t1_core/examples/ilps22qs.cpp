#include <iostream>
#include <unistd.h>

#include <tobas_t1_core/ilps22qs.hpp>

using namespace std;

int main()
{
  t1::ILPS22QS baro;
  double pres, temp;

  if (!baro.initialize()) {
    cerr << "Failed to initialize barometer." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!baro.readPressure(pres)) {
      cerr << "Failed to read pressure." << endl;
      return EXIT_FAILURE;
    }

    if (!baro.readTemperature(temp)) {
      cerr << "Failed to read temperature." << endl;
      return EXIT_FAILURE;
    }

    cout << "Pressure [Pa]     : " << pres << endl;
    cout << "Temperature [degC]: " << temp << endl;

    sleep(1);
  }

  return EXIT_SUCCESS;
}
