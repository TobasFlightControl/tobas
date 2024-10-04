#include <iostream>
#include <unistd.h>

#include <tobas_aso_core/sbus.hpp>

using namespace std;

void onPacket(const aso::SBUS::Packet& packet)
{
  for (size_t ch = 0; ch < aso::SBUS::kChannelSize; ++ch)
    cout << "Channel " << ch << ": " << packet.periods.at(ch) << endl;
  cout << endl;
}

int main()
{
  aso::SBUS sbus(&onPacket);

  if (!sbus.initialize())
  {
    cerr << "Failed to initialize S.BUS driver." << endl;
    return EXIT_FAILURE;
  }
  cout << "S.BUS driver is initialized." << endl;

  sbus.spin();

  return EXIT_SUCCESS;
}
