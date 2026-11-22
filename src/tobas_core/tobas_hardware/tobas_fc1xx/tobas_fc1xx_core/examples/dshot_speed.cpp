// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_fc1xx_core/dshot.hpp"

using namespace std::chrono_literals;

int main(int _argc, char* _argv[])
{
  // Parse arguments.
  if (_argc != 7) {
    std::cerr << "Usage: " << _argv[0] << " <Channel> <KV> <Prop Diameter> <Poles> <Gain> <Target RPM>\n";
    return EXIT_FAILURE;
  }
  const auto channel = std::stoul(_argv[1]);
  const auto kv = std::stoul(_argv[2]);  // [rpm/V]
  const auto d = std::stoul(_argv[3]);   // [inch]
  const auto poles = std::stoul(_argv[4]);
  const auto gain = std::stoul(_argv[5]);
  const auto tar_rpm = std::stoul(_argv[6]);
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

  // Set parameters.
  dshot.setKv(channel, tobas::st::rpm2rps(kv));
  if (const auto result = dshot.transfer(); !result) {
    std::cerr << "Failed to set Kv: " << result.error() << '\n';
    return EXIT_FAILURE;
  }
  std::this_thread::sleep_for(1ms);

  dshot.setInternalResistance(channel, 0.25);
  if (const auto result = dshot.transfer(); !result) {
    std::cerr << "Failed to set internal resistance: " << result.error() << '\n';
    return EXIT_FAILURE;
  }
  std::this_thread::sleep_for(1ms);

  dshot.setPropellerDiameter(channel, tobas::st::inch2meter(d));
  if (const auto result = dshot.transfer(); !result) {
    std::cerr << "Failed to set propeller diameter: " << result.error() << '\n';
    return EXIT_FAILURE;
  }
  std::this_thread::sleep_for(1ms);

  dshot.setMomentConstant(channel, 2e-4);
  if (const auto result = dshot.transfer(); !result) {
    std::cerr << "Failed to set moment constant: " << result.error() << '\n';
    return EXIT_FAILURE;
  }
  std::this_thread::sleep_for(1ms);

  dshot.setNumPoles(channel, poles);
  if (const auto result = dshot.transfer(); !result) {
    std::cerr << "Failed to set the number of poles: " << result.error() << '\n';
    return EXIT_FAILURE;
  }
  std::this_thread::sleep_for(1ms);

  dshot.setRpmControlGain(channel, gain);
  if (const auto result = dshot.transfer(); !result) {
    std::cerr << "Failed to set the speed control gain: " << result.error() << '\n';
    return EXIT_FAILURE;
  }
  std::this_thread::sleep_for(1ms);

  // Command target speed.
  dshot.setTargetSpeed(channel, tobas::st::rpm2rps(tar_rpm));
  while (true) {
    if (const auto result = dshot.transfer(); !result) {
      std::cerr << "Failed to set target speed: " << result.error() << '\n';
      continue;
    }
    dshot.printCurrentState(channel);
    std::this_thread::sleep_for(10ms);
  }
}
