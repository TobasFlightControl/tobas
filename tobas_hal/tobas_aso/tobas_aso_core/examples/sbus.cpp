#include <iostream>
#include <unistd.h>

#include <tobas_aso_core/sbus.hpp>

using namespace std;

int main()
{
  aso::SBUS sbus;

  if (!sbus.initialize())
  {
    cerr << "Failed to initialize S.BUS driver." << endl;
    return EXIT_FAILURE;
  }
  cout << "S.BUS driver is initialized." << endl;

  while (true)
  {
    switch (sbus.update())
    {
      case aso::SBUS::ERROR:
        cerr << "Failed to update S.BUS message." << endl;
        return EXIT_FAILURE;
      case aso::SBUS::THROTTLE:
        for (size_t ch = 0; ch < aso::SBUS::kChannelSize; ++ch)
          cout << "Channel " << ch << ": " << sbus.getPeriod(ch) << endl;
        cout << endl;
        break;
      case aso::SBUS::TELEMETRY:
        break;
      default:
        cerr << "Unknown message type received." << endl;
        return false;
    }
  }

  return EXIT_SUCCESS;
}
