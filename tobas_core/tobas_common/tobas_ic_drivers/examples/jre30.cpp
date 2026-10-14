// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_ic_drivers/jre30.hpp>

using namespace std;

void onPacket(shared_ptr<const tobas::driver::JRE30Packet> _packet)
{
  const auto packet = dynamic_pointer_cast<const tobas::driver::JRE30Packet_A>(_packet);
  if (!packet) {
    cerr << "Failed to cast JRE30 packet to type A.\n";
    return;
  }

  cout << "Protocol Version: " << (int)packet->protocol_version << '\n';
  cout << "Frame Count: " << (int)packet->frame_count << '\n';
  cout << "Distance [m]: " << packet->distance << '\n';
  cout << "Strength: " << packet->strength << '\n';

  // Status
  cout << boolalpha;
  cout << "Gain: " << packet->gain << '\n';
  cout << "NTRK: " << packet->ntrk << '\n';
  cout << "Fail: " << packet->fail << '\n';
  cout << noboolalpha;

  cout << "----------\n";
}

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    cerr << "Usage: " << _argv[0] << " <UART Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::driver::JRE30 jre30(&onPacket);

  if (!jre30.initialize(device)) {
    cerr << "Failed to initialize JRE30 driver.\n";
    return EXIT_FAILURE;
  }
  cout << "JRE30 driver has been initialized.\n";

  jre30.start();
  jre30.spin();

  return EXIT_SUCCESS;
}
