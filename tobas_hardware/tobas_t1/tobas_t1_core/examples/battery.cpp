#include <iostream>
#include <thread>

#include <tobas_t1_core/battery.hpp>

using namespace std;

int main()
{
  t1::Battery battery;
  float voltage, current;

  if (!battery.initialize()) {
    cerr << "Failed to initialize ADC." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!battery.read(voltage, current)) {
      cerr << "Failed to read battery status." << endl;
      return EXIT_FAILURE;
    }

    cout << "Voltage [V]: " << voltage << endl;
    cout << "Current [A]: " << current << endl;

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
