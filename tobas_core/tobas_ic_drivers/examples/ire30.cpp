#include <iostream>
#include <unistd.h>

#include <tobas_ic_drivers/jre30.hpp>

using namespace std;

void onPacket(const driver::JRE30::Packet& packet)
{
  cout << "Protocol Version: " << packet.protocol_version << endl;
  cout << "Frame Count: " << packet.frame_count << endl;
  cout << "Distance [m]: " << packet.distance << endl;
  cout << "Strength: " << packet.strength << endl;

  // Status
  cout << boolalpha;
  cout << "Gain: " << packet.gain << endl;
  cout << "NTRK: " << packet.ntrk << endl;
  cout << "Fail: " << packet.fail << endl;
  cout << noboolalpha;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <Device>" << endl;
    return EXIT_FAILURE;
  }

  driver::JRE30 jre30(&onPacket);

  if (!jre30.initialize(argv[1]))
  {
    cerr << "Failed to initialize JRE30 driver." << endl;
    return EXIT_FAILURE;
  }
  cout << "JRE30 driver is initialized." << endl;

  jre30.start();
  jre30.spin();

  return EXIT_SUCCESS;
}
