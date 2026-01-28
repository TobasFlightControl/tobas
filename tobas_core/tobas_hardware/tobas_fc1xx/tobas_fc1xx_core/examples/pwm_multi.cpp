#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/pwm.hpp"

using namespace std;

int main()
{
  fc1xx::PWM pwm;
  constexpr uint16_t periods[] = { 0, 100, 200, 400, 800, 1200, 1600, 2000 };

  if (!pwm.initialize()) {
    cerr << "Failed to initialize PWM driver." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    for (size_t ch = 0; ch < fc1xx::PWM::kChannelSize; ++ch) {
      if (!pwm.setPeriod(ch, periods[ch])) {
        cerr << "Failed to set PWM period of channel " << ch << "." << endl;
        continue;
      }
    }

    if (!pwm.transfer()) {
      cerr << "Failed to command PWM periods." << endl;
      continue;
    }

    this_thread::sleep_for(100ms);
  }

  return EXIT_SUCCESS;
}
