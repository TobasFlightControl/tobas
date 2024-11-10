#include <iostream>
#include <unistd.h>

#include <tobas_ic_drivers/jre30.hpp>

using namespace std;

void onPacket(shared_ptr<const driver::JRE30Packet> _packet)
{
  const auto packet = dynamic_pointer_cast<const driver::JRE30Packet_A>(_packet);
  if (packet == nullptr)
  {
    cerr << "Failed to cast JRE30 packet to type A." << endl;
    return;
  }

  cout << "Protocol Version: " << (int)packet->protocol_version << endl;
  cout << "Frame Count: " << (int)packet->frame_count << endl;
  cout << "Distance [m]: " << packet->distance << endl;
  cout << "Strength: " << packet->strength << endl;

  // Status
  cout << boolalpha;
  cout << "Gain: " << packet->gain << endl;
  cout << "NTRK: " << packet->ntrk << endl;
  cout << "Fail: " << packet->fail << endl;
  cout << noboolalpha;

  cout << "----------" << endl;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];

  driver::JRE30 jre30(&onPacket);

  if (!jre30.initialize(device))
  {
    cerr << "Failed to initialize JRE30 driver." << endl;
    return EXIT_FAILURE;
  }
  cout << "JRE30 driver is initialized." << endl;

  jre30.start();
  jre30.spin();

  return EXIT_SUCCESS;
}
