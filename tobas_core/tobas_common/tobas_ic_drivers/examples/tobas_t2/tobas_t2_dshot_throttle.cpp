// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <tobas_ic_drivers/tobas_t2/dshot.hpp>

using namespace std;

int main(int _argc, char** _argv)
{
  // Parse arguments.
  if (_argc != 4) {
    cerr << "Usage: " << _argv[0] << " <SPI Device> <Channel> <Throttle>\n";
    return EXIT_FAILURE;
  }
  const auto device = _argv[1];
  const auto channel = stoul(_argv[2]);
  const auto throttle = stoul(_argv[3]);

  if (channel >= tobas::t2::DShot::kChannelSize) {
    cerr << "DShot channel is out of range.\n";
    return EXIT_FAILURE;
  }

  // Initialize driver.
  tobas::t2::DShot dshot;
  if (!dshot.initialize(device)) {
    cerr << "Failed to initialize DShot driver.\n";
    return EXIT_FAILURE;
  }

  // Command throttle.
  dshot.setThrottle(channel, throttle);
  while (true) {
    if (const auto result = dshot.transfer(); !result) {
      cerr << "Failed to command DShot throttles: " << result.error() << '\n';
      continue;
    }
    dshot.printCurrentState(channel);
    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
