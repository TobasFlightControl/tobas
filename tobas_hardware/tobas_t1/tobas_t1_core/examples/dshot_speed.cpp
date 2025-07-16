#include <cmath>
#include <iostream>
#include <thread>

#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_t1_core/dshot.hpp"

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 6) {
    cerr << "Usage: " << argv[0] << " <Channel> <KV> <Prop Diameter> <Poles> <Target RPM>" << endl;
    return EXIT_FAILURE;
  }
  const auto channel = stoi(argv[1]);
  const auto kv = stoi(argv[2]);  // [rpm/V]
  const auto d = stoi(argv[3]);   // [inch]
  const auto poles = stoi(argv[3]);
  const auto tar_rpm = stoi(argv[4]);

  t1::DShot dshot;

  if (!dshot.initialize()) {
    throw runtime_error("Failed to initialize DShot driver.");
  }

  if (!dshot.setKv(channel, tobas_std::rpm2rps(kv))) {
    throw runtime_error("Failed to set Kv.");
  }
  if (!dshot.transfer()) {
    throw runtime_error("Failed to send Kv.");
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setInternalResistance(channel, 0.25)) {  // Typical value
    throw runtime_error("Failed to set internal resistance.");
  }
  if (!dshot.transfer()) {
    throw runtime_error("Failed to send internal resistance.");
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setPropellerDiameter(channel, tobas_std::meter2inch(d))) {
    throw runtime_error("Failed to set propeller diameter.");
  }
  if (!dshot.transfer()) {
    throw runtime_error("Failed to send propeller diameter.");
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setMomentConstant(channel, 2e-4)) {  // Typical value
    throw runtime_error("Failed to set moment constant.");
  }
  if (!dshot.transfer()) {
    throw runtime_error("Failed to send moment constant.");
  }
  this_thread::sleep_for(1ms);

  if (!dshot.setNumPoles(channel, poles)) {
    throw runtime_error("Failed to set the number of poles.");
  }
  if (!dshot.transfer()) {
    throw runtime_error("Failed to send the number of poles.");
  }
  this_thread::sleep_for(1ms);

  while (true) {
    if (!dshot.setTargetSpeed(channel, tobas_std::rpm2rps(tar_rpm))) {
      throw runtime_error("Failed to set target speed.");
    }

    if (!dshot.transfer()) {
      throw runtime_error("Failed to send target speed.");
    }

    dshot.printCurrentState(channel);

    this_thread::sleep_for(10ms);
  }
}
