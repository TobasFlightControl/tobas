#include <iostream>
#include <unistd.h>

#include <tobas_ic_drivers/bmm150.hpp>

using namespace std;

int main()
{
  driver::BMM150 mag;
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

    cout << "Magnetic Field [μT]: " << mx << ", " << my << ", " << mz << endl;

    sleep(1);
  }

  return EXIT_SUCCESS;
}
