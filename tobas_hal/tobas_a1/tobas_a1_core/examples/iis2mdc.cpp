#include <iostream>
#include <unistd.h>

#include <tobas_a1_core/iis2mdc.hpp>

using namespace std;

int main()
{
  a1::IIS2MDC mag;
  double mx, my, mz;

  if (!mag.initialize())
  {
    cerr << "Failed to initialize magnetometer." << endl;
    return EXIT_FAILURE;
  }

  while (true)
  {
    if (!mag.readMag(mx, my, mz))
    {
      cerr << "Failed to read magnetic field." << endl;
      return EXIT_FAILURE;
    }

    cout << "Magnetic Field [gauss]: " << mx << ", " << my << ", " << mz << endl;
    sleep(1);
  }

  return EXIT_SUCCESS;
}
