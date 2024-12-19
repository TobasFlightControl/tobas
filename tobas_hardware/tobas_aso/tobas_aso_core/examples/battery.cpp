#include <iostream>
#include <unistd.h>

#include <tobas_aso_core/dshot.hpp>

using namespace std;

int main()
{
  aso::DShot dshot;

  if (!dshot.initialize())
    return EXIT_FAILURE;

  while (true)
  {
    if (!dshot.transfer())
      return EXIT_FAILURE;

    cout << "Voltage [V]: " << dshot.getBatteryVoltage() << endl;
    cout << "Current [A]: " << dshot.getBatteryCurrent() << endl;

    sleep(1);
  }

  return EXIT_SUCCESS;
}
