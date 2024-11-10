#include <iostream>
#include <unistd.h>

#include <tobas_ic_drivers/sbus.hpp>

using namespace std;

void onPacket(const driver::SBUS::Packet& packet)
{
  for (size_t ch = 0; ch < driver::SBUS::kChannelSize; ++ch)
    cout << "Channel " << ch << ": " << packet.periods.at(ch) << endl;
  cout << endl;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];

  driver::SBUS sbus(&onPacket);

  if (!sbus.initialize(device))
  {
    cerr << "Failed to initialize S.BUS driver." << endl;
    return EXIT_FAILURE;
  }
  cout << "S.BUS driver is initialized." << endl;

  sbus.start();
  sbus.spin();

  return EXIT_SUCCESS;
}
