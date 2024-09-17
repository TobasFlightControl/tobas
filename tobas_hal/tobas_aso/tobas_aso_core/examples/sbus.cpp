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
    if (!sbus.update())
    {
      cerr << "Failed to update S.BUS message." << endl;
      return EXIT_FAILURE;
    }

    for (size_t ch = 0; ch < aso::SBUS::kChannelSize; ++ch)
      cout << "Channel " << ch << ": " << sbus.getPeriod(ch) << endl;
    cout << endl;
  }

  return EXIT_SUCCESS;
}
