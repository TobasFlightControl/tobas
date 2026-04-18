#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

#include <tobas_ic_drivers/bmm350.hpp>

using namespace std;

int main()
{
  tobas::driver::BMM350 mag;
  double mx, my, mz;
  if (!mag.initialize()) {
    std::cerr << "BMM350 initialize failed." << std::endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!mag.readMag(mx, my, mz)) {
      cerr << "Failed to read magnetic field." << endl;
      return EXIT_FAILURE;
    }
    const double norm = sqrt(mx * mx + my * my + mz * mz);
    cout << "Magnetic Field [μT]: " << mx << ", " << my << ", " << mz << "| norm=" << norm << endl;

    this_thread::sleep_for(1s);
  }
}
