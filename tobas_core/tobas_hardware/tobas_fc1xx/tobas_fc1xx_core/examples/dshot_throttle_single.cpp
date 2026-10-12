// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/dshot.hpp"

using namespace std;

int main(int _argc, char** _argv)
{
  // Parse arguments.
  if (_argc != 3) {
    cerr << "Usage: " << _argv[0] << " <Channel> <Throttle>" << endl;
    return EXIT_FAILURE;
  }
  const auto channel = stoul(_argv[1]);
  const auto throttle = stoul(_argv[2]);
  if (channel >= tobas::fc1xx::DShot::kChannelSize) {
    cerr << "DShot channel is out of range." << endl;
    return EXIT_FAILURE;
  }

  // Initialize driver.
  tobas::fc1xx::DShot dshot;
  if (!dshot.initialize()) {
    cerr << "Failed to initialize DShot driver." << endl;
    return EXIT_FAILURE;
  }

  // Command throttle.
  dshot.setThrottle(channel, throttle);
  while (true) {
    if (const auto result = dshot.transfer(); !result) {
      cerr << "Failed to command DShot throttles: " << result.error() << endl;
      continue;
    }
    dshot.printCurrentState(channel);
    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
