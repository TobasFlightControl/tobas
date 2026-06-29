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
  if (argc != 7) {
    cerr << "Usage: " << argv[0] << " <Channel> <KV> <Prop Diameter> <Poles> <Gain> <Target RPM>" << endl;
    return EXIT_FAILURE;
  }
  const auto channel = stoi(argv[1]);
  const auto kv = stoi(argv[2]);  // [rpm/V]
  const auto d = stoi(argv[3]);   // [inch]
  const auto poles = stoi(argv[4]);
  const auto gain = stoi(argv[5]);
  const auto tar_rpm = stoi(argv[6]);

  tobas::fc1xx::DShot dshot;

  if (!dshot.initialize()) {
    cerr << "Failed to initialize DShot driver." << endl;
    return EXIT_FAILURE;
  }

  if (!dshot.setKv(channel, tobas::st::rpm2rps(kv))) {
    cerr << "Failed to set Kv." << endl;
    return EXIT_FAILURE;
  }
  if (!dshot.transfer()) {
    cerr << "Failed to send Kv." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setInternalResistance(channel, 0.25)) {  // Typical value
    cerr << "Failed to set internal resistance." << endl;
    return EXIT_FAILURE;
  }
  if (!dshot.transfer()) {
    cerr << "Failed to send internal resistance." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setPropellerDiameter(channel, tobas::st::inch2meter(d))) {
    cerr << "Failed to set propeller diameter." << endl;
    return EXIT_FAILURE;
  }
  if (!dshot.transfer()) {
    cerr << "Failed to send propeller diameter." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setMomentConstant(channel, 2e-4)) {  // Typical value
    cerr << "Failed to set moment constant." << endl;
    return EXIT_FAILURE;
  }
  if (!dshot.transfer()) {
    cerr << "Failed to send moment constant." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setNumPoles(channel, poles)) {
    cerr << "Failed to set the number of poles." << endl;
    return EXIT_FAILURE;
  }
  if (!dshot.transfer()) {
    cerr << "Failed to send the number of poles." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setRpmControlGain(channel, gain)) {
    cerr << "Failed to set the speed control gain." << endl;
    return EXIT_FAILURE;
  }
  if (!dshot.transfer()) {
    cerr << "Failed to send the speed control gain." << endl;
    return EXIT_FAILURE;
  }
  this_thread::sleep_for(1ms);

  while (true) {
    if (!dshot.setTargetSpeed(channel, tobas::st::rpm2rps(tar_rpm))) {
      cerr << "Failed to set target speed." << endl;
      continue;
    }

    if (!dshot.transfer()) {
      cerr << "Failed to send target speed." << endl;
      continue;
    }

    dshot.printCurrentState(channel);

    this_thread::sleep_for(10ms);
  }
}
