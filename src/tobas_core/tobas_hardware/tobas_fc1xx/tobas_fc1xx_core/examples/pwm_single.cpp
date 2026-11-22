// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "tobas_fc1xx_core/pwm.hpp"

using namespace std::chrono_literals;

int main(int _argc, char* _argv[])
{
  if (_argc != 3) {
    std::cerr << "Usage: " << _argv[0] << " <Channel> <Period>\n";
    return EXIT_FAILURE;
  }
  const std::size_t channel = std::stoul(_argv[1]);
  const uint16_t period = std::stoi(_argv[2]);

  if (channel >= tobas::fc1xx::PWM::kChannelSize) {
    std::cerr << "PWM channel out of range: " << channel << '\n';
    return EXIT_FAILURE;
  }

  tobas::fc1xx::PWM pwm;
  if (!pwm.initialize()) {
    std::cerr << "Failed to initialize PWM driver.\n";
    return EXIT_FAILURE;
  }

  pwm.setPeriod(channel, period);

  while (true) {
    if (!pwm.transfer()) {
      std::cerr << "Failed to command PWM periods.\n";
      continue;
    }
    std::this_thread::sleep_for(100ms);
  }

  return EXIT_SUCCESS;
}
