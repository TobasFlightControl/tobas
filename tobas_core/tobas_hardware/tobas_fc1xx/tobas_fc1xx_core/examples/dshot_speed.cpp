// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cmath>
#include <iostream>
#include <thread>

#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_fc1xx_core/dshot.hpp"

using namespace std;

int main(int argc, char** argv)
{
  // Parse arguments.
  if (argc != 7) {
    cerr << "Usage: " << argv[0] << " <Channel> <KV> <Prop Diameter> <Poles> <Gain> <Target RPM>" << endl;
    return EXIT_FAILURE;
  }
  const auto channel = stoul(argv[1]);
  const auto kv = stoul(argv[2]);  // [rpm/V]
  const auto d = stoul(argv[3]);   // [inch]
  const auto poles = stoul(argv[4]);
  const auto gain = stoul(argv[5]);
  const auto tar_rpm = stoul(argv[6]);
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

  // Set parameters.
  dshot.setKv(channel, tobas::st::rpm2rps(kv));
  if (!dshot.transfer()) {
    cerr << "Failed to set Kv." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  dshot.setInternalResistance(channel, 0.25);
  if (!dshot.transfer()) {
    cerr << "Failed to set internal resistance." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  dshot.setPropellerDiameter(channel, tobas::st::inch2meter(d));
  if (!dshot.transfer()) {
    cerr << "Failed to set propeller diameter." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  dshot.setMomentConstant(channel, 2e-4);
  if (!dshot.transfer()) {
    cerr << "Failed to set moment constant." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  dshot.setNumPoles(channel, poles);
  if (!dshot.transfer()) {
    cerr << "Failed to set the number of poles." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  dshot.setRpmControlGain(channel, gain);
  if (!dshot.transfer()) {
    cerr << "Failed to set the speed control gain." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  // Command target speed.
  dshot.setTargetSpeed(channel, tobas::st::rpm2rps(tar_rpm));
  while (true) {
    if (!dshot.transfer()) {
      cerr << "Failed to set target speed." << endl;
      continue;
    }
    dshot.printCurrentState(channel);
    this_thread::sleep_for(10ms);
  }
}
