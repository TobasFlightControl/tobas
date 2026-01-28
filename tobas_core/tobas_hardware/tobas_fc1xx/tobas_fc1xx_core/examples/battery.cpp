#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/battery.hpp"

using namespace std;

int main()
{
  fc1xx::Battery battery;
  float voltage, current;

  if (!battery.initialize()) {
    cerr << "Failed to initialize ADC." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!battery.read(voltage, current)) {
      cerr << "Failed to read battery status." << endl;
      continue;
    }

    cout << "Voltage [V]: " << voltage << endl;
    cout << "Current [A]: " << current << endl;

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
