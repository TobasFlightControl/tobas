// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstddef>
#include <cstdlib>
#include <ios>
#include <iostream>

#include <tobas_sbus_driver/sbus.hpp>

/** Print a received sensor packet. */
void onPacket(const tobas::SBUS::Packet& _packet)
{
  for (std::size_t ch = 0; ch < tobas::SBUS::kChannelSize; ++ch) {
    std::cout << "Channel " << ch << ": " << _packet.periods.at(ch) << '\n';
  }
  std::cout << "-----\n";

  std::cout << std::boolalpha;
  std::cout << "Channel 17: " << _packet.ch17 << '\n';
  std::cout << "Channel 18: " << _packet.ch18 << '\n';
  std::cout << "Frame Lost: " << _packet.frame_lost << '\n';
  std::cout << "Fail-Safe: " << _packet.failsafe << '\n';
  std::cout << std::noboolalpha;

  std::cout << '\n';
}

int main(int _argc, char* _argv[])
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::SBUS sbus(&onPacket);

  if (!sbus.initialize(device)) {
    std::cerr << "Failed to initialize S.BUS driver.\n";
    return EXIT_FAILURE;
  }
  std::cout << "S.BUS driver has been initialized.\n";

  sbus.start();
  sbus.spin();

  return EXIT_SUCCESS;
}
