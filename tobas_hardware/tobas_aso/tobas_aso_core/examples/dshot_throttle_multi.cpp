#include <iostream>
#include <thread>

#include <tobas_aso_core/dshot.hpp>

using namespace std;

int main()
{
  aso::DShot dshot;
  constexpr uint16_t throttles[] = { 0, 1, 2, 47, 48, 1023, 1024, 2047 };

  if (!dshot.initialize())
  {
    cerr << "Failed to initialize DShot driver." << endl;
    return EXIT_FAILURE;
  }

  while (true)
  {
    for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
    {
      if (!dshot.setThrottle(ch, throttles[ch]))
      {
        cerr << "Failed to set DShot throttle of channel " << ch << "." << endl;
        return EXIT_FAILURE;
      }
    }

    if (!dshot.transfer())
    {
      cerr << "Failed to command DShot throttles." << endl;
      return EXIT_FAILURE;
    }

    dshot.printCurrentStates();
    cout << "----------" << endl;

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
