// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "tobas_fc1xx_core/dshot.hpp"

using namespace std::chrono_literals;

int main(int _argc, char* _argv[])
{
  // Parse arguments.
  if (_argc != 3) {
    std::cerr << "Usage: " << _argv[0] << " <Channel> <Throttle>\n";
    return EXIT_FAILURE;
  }
  const auto channel = std::stoul(_argv[1]);
  const auto throttle = std::stoul(_argv[2]);
  if (channel >= tobas::fc1xx::DShot::kChannelSize) {
    std::cerr << "DShot channel is out of range.\n";
    return EXIT_FAILURE;
  }

  // Initialize driver.
  tobas::fc1xx::DShot dshot;
  if (!dshot.initialize()) {
    std::cerr << "Failed to initialize DShot driver.\n";
    return EXIT_FAILURE;
  }

  // Command throttle.
  dshot.setThrottle(channel, throttle);
  while (true) {
    if (const auto result = dshot.transfer(); !result) {
      std::cerr << "Failed to command DShot throttles: " << result.error() << '\n';
      continue;
    }
    dshot.printCurrentState(channel);
    std::this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
