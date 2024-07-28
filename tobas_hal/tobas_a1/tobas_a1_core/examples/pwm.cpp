#include <iostream>
#include <unistd.h>

#include <tobas_a1_core/pwm.hpp>

using namespace std;

int main()
{
  a1::PWM pwm;
  constexpr uint16_t periods[] = { 0, 100, 200, 400, 800, 1200, 1600, 2000 };

  if (!pwm.initialize())
  {
    cerr << "Failed to initialize PWM driver." << endl;
    return EXIT_FAILURE;
  }

  while (true)
  {
    for (size_t ch = 0; ch < a1::PWM::kChannelSize; ++ch)
    {
      if (!pwm.setPeriod(ch, periods[ch]))
      {
        cerr << "Failed to set PWM period of channel " << ch << "." << endl;
        return EXIT_FAILURE;
      }
    }

    if (!pwm.transfer())
    {
      cerr << "Failed to command PWM periods." << endl;
      return EXIT_FAILURE;
    }

    usleep(10000);
  }

  return EXIT_SUCCESS;
}
