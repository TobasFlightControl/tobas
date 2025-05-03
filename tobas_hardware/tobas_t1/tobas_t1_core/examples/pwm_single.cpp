#include <iostream>
#include <thread>

#include <tobas_t1_core/pwm.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3) {
    cerr << "Usage: " << argv[0] << " <Channel> <Period>" << endl;
    return EXIT_FAILURE;
  }
  const size_t channel = stoul(argv[1]);
  const uint16_t period = stoi(argv[2]);

  t1::PWM pwm;
  if (!pwm.initialize()) {
    cerr << "Failed to initialize PWM driver." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!pwm.setPeriod(channel, period)) {
      cerr << "Failed to set PWM period of channel " << channel << "." << endl;
      return EXIT_FAILURE;
    }

    if (!pwm.transfer()) {
      cerr << "Failed to command PWM periods." << endl;
      return EXIT_FAILURE;
    }

    this_thread::sleep_for(100ms);
  }

  return EXIT_SUCCESS;
}
