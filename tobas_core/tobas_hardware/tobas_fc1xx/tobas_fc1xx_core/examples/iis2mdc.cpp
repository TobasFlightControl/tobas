#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/iis2mdc.hpp"

using namespace std;
using namespace std::chrono_literals;

int main()
{
  fc1xx::IIS2MDC mag;
  double mx, my, mz;

  if (!mag.initialize()) {
    cerr << "Failed to initialize magnetometer." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!mag.readMag(mx, my, mz)) {
      cerr << "Failed to read magnetic field." << endl;
      return EXIT_FAILURE;
    }

    cout << "Magnetic Field [gauss]: " << mx << ", " << my << ", " << mz << endl;

    this_thread::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
