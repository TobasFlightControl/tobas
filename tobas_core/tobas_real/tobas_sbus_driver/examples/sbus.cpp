// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_sbus_driver/sbus.hpp>

using namespace std;

void onPacket(const tobas::SBUS::Packet& _packet)
{
  for (size_t ch = 0; ch < tobas::SBUS::kChannelSize; ++ch) {
    cout << "Channel " << ch << ": " << _packet.periods.at(ch) << endl;
  }
  cout << "-----" << endl;

  cout << boolalpha;
  cout << "Channel 17: " << _packet.ch17 << endl;
  cout << "Channel 18: " << _packet.ch18 << endl;
  cout << "Frame Lost: " << _packet.frame_lost << endl;
  cout << "Fail-Safe: " << _packet.failsafe << endl;
  cout << noboolalpha;

  cout << endl;
}

int main(int _argc, char** _argv)
{
  if (_argc != 2) {
    cerr << "Usage: " << _argv[0] << " <Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::SBUS sbus(&onPacket);

  if (!sbus.initialize(device)) {
    cerr << "Failed to initialize S.BUS driver." << endl;
    return EXIT_FAILURE;
  }
  cout << "S.BUS driver has been initialized." << endl;

  sbus.start();
  sbus.spin();

  return EXIT_SUCCESS;
}
