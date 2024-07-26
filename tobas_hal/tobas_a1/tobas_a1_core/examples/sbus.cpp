#include <iostream>
#include <unistd.h>

#include <tobas_a1_core/sbus.hpp>

using namespace std;

int main()
{
  a1::SBUS sbus;

  if (!sbus.initialize())
  {
    cerr << "Failed to initialize S.BUS driver." << endl;
    return EXIT_FAILURE;
  }

  while (true)
  {
    if (!sbus.update())
    {
      cerr << "Failed to update S.BUS message." << endl;
      return EXIT_FAILURE;
    }

    for (size_t ch = 0; ch < a1::SBUS::kChannelSize; ++ch)
      cout << "Channel " << ch << ": " << sbus.getPeriod(ch) << endl;

    sleep(1);
  }

  return EXIT_SUCCESS;
}
