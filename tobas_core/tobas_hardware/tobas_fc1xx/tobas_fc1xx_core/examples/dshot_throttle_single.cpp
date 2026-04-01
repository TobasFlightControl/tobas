// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/dshot.hpp"

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3) {
    cerr << "Usage: " << argv[0] << " <Channel> <Throttle>" << endl;
    return EXIT_FAILURE;
  }
  const size_t channel = stoul(argv[1]);
  const uint16_t throttle = stoi(argv[2]);

  tobas::fc1xx::DShot dshot;
  if (!dshot.initialize()) {
    cerr << "Failed to initialize DShot driver." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    if (!dshot.setThrottle(channel, throttle)) {
      cerr << "Failed to set DShot throttle of channel " << channel << "." << endl;
      continue;
    }

    if (!dshot.transfer()) {
      cerr << "Failed to command DShot throttles." << endl;
      continue;
    }

    dshot.printCurrentState(channel);

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
