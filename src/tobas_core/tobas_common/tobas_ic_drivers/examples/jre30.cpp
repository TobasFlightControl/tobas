// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <ios>
#include <iostream>
#include <memory>

#include <tobas_ic_drivers/jre30.hpp>

/** Print a received sensor packet. */
void onPacket(const std::shared_ptr<const tobas::driver::JRE30Packet>& _packet)
{
  const auto packet = std::dynamic_pointer_cast<const tobas::driver::JRE30Packet_A>(_packet);
  if (!packet) {
    std::cerr << "Failed to cast JRE30 packet to type A.\n";
    return;
  }

  std::cout << "Protocol Version: " << static_cast<int>(packet->protocol_version) << '\n';
  std::cout << "Frame Count: " << static_cast<int>(packet->frame_count) << '\n';
  std::cout << "Distance [m]: " << packet->distance << '\n';
  std::cout << "Strength: " << packet->strength << '\n';

  // Status
  std::cout << std::boolalpha;
  std::cout << "Gain: " << packet->gain << '\n';
  std::cout << "NTRK: " << packet->ntrk << '\n';
  std::cout << "Fail: " << packet->fail << '\n';
  std::cout << std::noboolalpha;

  std::cout << "----------\n";
}

int main(int _argc, char* _argv[])
{
  if (_argc != 2) {
    std::cerr << "Usage: " << _argv[0] << " <UART Device>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];

  tobas::driver::JRE30 jre30(&onPacket);

  if (!jre30.initialize(device)) {
    std::cerr << "Failed to initialize JRE30 driver.\n";
    return EXIT_FAILURE;
  }
  std::cout << "JRE30 driver has been initialized.\n";

  jre30.start();
  jre30.spin();

  return EXIT_SUCCESS;
}
