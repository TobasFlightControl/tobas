#include <cmath>
#include <iostream>
#include <thread>

#include <tobas_aso_core/dshot.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 4)
  {
    cerr << "Usage: " << argv[0] << " <Channel> <Gain> <Target RPM>" << endl;
    return EXIT_FAILURE;
  }
  const size_t channel = stoi(argv[1]);
  const uint8_t gain = stoi(argv[2]);
  const uint16_t tar_rpm = stoi(argv[3]);

  aso::DShot dshot;

  if (!dshot.initialize())
    throw runtime_error("Failed to initialize DShot driver.");

  if (!dshot.setKv(channel, 920 * (M_PI / 30)))
    throw runtime_error("Failed to set Kv.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send Kv.");
  this_thread::sleep_for(1ms);

  if (!dshot.setInternalResistance(channel, 0.25))
    throw runtime_error("Failed to set internal resistance.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send internal resistance.");
  this_thread::sleep_for(1ms);

  if (!dshot.setPropellerDiameter(channel, 9 * 0.0254))
    throw runtime_error("Failed to set propeller diameter.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send propeller diameter.");
  this_thread::sleep_for(1ms);

  if (!dshot.setMomentConstant(channel, 5.442e-5))
    throw runtime_error("Failed to set moment constant.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send moment constant.");
  this_thread::sleep_for(1ms);

  if (!dshot.setNumPoles(channel, 14))
    throw runtime_error("Failed to set the number of poles.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send the number of poles.");
  this_thread::sleep_for(1ms);

  if (!dshot.setSpeedControlGain(channel, gain))
    throw runtime_error("Failed to set the speed control gain.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send the speed control gain.");
  this_thread::sleep_for(1ms);

  while (true)
  {
    if (!dshot.setTargetSpeed(channel, tar_rpm * (M_PI / 30)))
      throw runtime_error("Failed to set target speed.");

    if (!dshot.transfer())
      throw runtime_error("Failed to send target speed.");

    dshot.printCurrentState(channel);

    this_thread::sleep_for(100ms);
  }
}
