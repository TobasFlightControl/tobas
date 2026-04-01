// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/dshot.hpp"

using namespace std;

int main()
{
  tobas::fc1xx::DShot dshot;
  constexpr uint16_t throttles[] = { 0, 1, 2, 47, 48, 1023, 1024, 2047 };

  if (!dshot.initialize()) {
    cerr << "Failed to initialize DShot driver." << endl;
    return EXIT_FAILURE;
  }

  while (true) {
    for (size_t ch = 0; ch < tobas::fc1xx::DShot::kChannelSize; ++ch) {
      if (!dshot.setThrottle(ch, throttles[ch])) {
        cerr << "Failed to set DShot throttle of channel " << ch << "." << endl;
        continue;
      }
    }

    if (!dshot.transfer()) {
      cerr << "Failed to command DShot throttles." << endl;
      continue;
    }

    dshot.printCurrentStates();
    cout << "----------" << endl;

    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
