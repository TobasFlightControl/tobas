// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_sbus_driver/sbus.hpp>

using namespace std;

void onPacket(const tobas::SBUS::Packet& _packet)
{
  for (size_t ch = 0; ch < tobas::SBUS::kChannelSize; ++ch) {
    cout << "Channel " << ch << ": " << _packet.periods.at(ch) << '\n';
  }
  cout << "-----\n";

  cout << boolalpha;
  cout << "Channel 17: " << _packet.ch17 << '\n';
  cout << "Channel 18: " << _packet.ch18 << '\n';
  cout << "Frame Lost: " << _packet.frame_lost << '\n';
  cout << "Fail-Safe: " << _packet.failsafe << '\n';
  cout << noboolalpha;

  cout << '\n';
}

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    cerr << "Usage: " << _argv[0] << " <Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::SBUS sbus(&onPacket);

  if (!sbus.initialize(device)) {
    cerr << "Failed to initialize S.BUS driver.\n";
    return EXIT_FAILURE;
  }
  cout << "S.BUS driver has been initialized.\n";

  sbus.start();
  sbus.spin();

  return EXIT_SUCCESS;
}
