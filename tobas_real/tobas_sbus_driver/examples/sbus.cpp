#include <iostream>

#include <tobas_sbus_driver/sbus.hpp>

using namespace std;

void onPacket(const tobas::SBUS::Packet& packet)
{
  for (size_t ch = 0; ch < tobas::SBUS::kChannelSize; ++ch) {
    cout << "Channel " << ch << ": " << packet.periods.at(ch) << endl;
  }
  cout << endl;
}

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];

  tobas::SBUS sbus(&onPacket);

  if (!sbus.initialize(device)) {
    cerr << "Failed to initialize S.BUS driver." << endl;
    return EXIT_FAILURE;
  }
  cout << "S.BUS driver is initialized." << endl;

  sbus.start();
  sbus.spin();

  return EXIT_SUCCESS;
}
