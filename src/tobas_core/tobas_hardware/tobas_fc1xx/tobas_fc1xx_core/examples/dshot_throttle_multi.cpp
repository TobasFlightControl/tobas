// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <ranges>
#include <thread>

#include "tobas_fc1xx_core/dshot.hpp"

using namespace std::chrono_literals;

int main()
{
  // Initialize driver.
  tobas::fc1xx::DShot dshot;
  if (!dshot.initialize()) {
    std::cerr << "Failed to initialize DShot driver.\n";
    return EXIT_FAILURE;
  }

  // Set throttles.
  for (const auto [ch, throttle] : std::views::enumerate(std::array{ 0, 1, 2, 47, 48, 1023, 1024, 2047 })) {
    dshot.setThrottle(ch, throttle);
  }

  // Command throttles.
  while (true) {
    if (const auto result = dshot.transfer(); !result) {
      std::cerr << "Failed to command DShot throttles: " << result.error() << '\n';
      continue;
    }
    dshot.printCurrentStates();
    std::cout << "----------\n";
    std::this_thread::sleep_for(10ms);
  }

  return EXIT_SUCCESS;
}
