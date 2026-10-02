// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>
#include <thread>

#include "tobas_fc1xx_core/dshot.hpp"

using namespace std;

int main()
{
  // Initialize driver.
  tobas::fc1xx::DShot dshot;
  if (!dshot.initialize()) {
    cerr << "Failed to initialize DShot driver." << endl;
    return EXIT_FAILURE;
  }

  // Set throttles.
  constexpr uint16_t throttles[] = { 0, 1, 2, 47, 48, 1023, 1024, 2047 };
  for (size_t ch = 0; ch < tobas::fc1xx::DShot::kChannelSize; ++ch) {
    dshot.setThrottle(ch, throttles[ch]);
  }

  // Command throttles.
  while (true) {
    if (const auto result = dshot.transfer(); !result) {
      cerr << "Failed to command DShot throttles: " << result.error() << endl;
      continue;
    }
    dshot.printCurrentStates();
    cout << "----------" << endl;
    this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
