// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <ranges>
#include <thread>

#include "tobas_fc1xx_core/pwm.hpp"

using namespace std::chrono_literals;

int main()
{
  tobas::fc1xx::PWM pwm;
  if (!pwm.initialize()) {
    std::cerr << "Failed to initialize PWM driver.\n";
    return EXIT_FAILURE;
  }

  while (true) {
    for (const auto [ch, period] : std::views::enumerate(std::array{ 0, 100, 200, 400, 800, 1200, 1600, 2000 })) {
      pwm.setPeriod(ch, period);
    }

    if (!pwm.transfer()) {
      std::cerr << "Failed to command PWM periods.\n";
      continue;
    }

    std::this_thread::sleep_for(100ms);
  }

  return EXIT_SUCCESS;
}
