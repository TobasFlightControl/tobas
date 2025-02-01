#include <cmath>
#include <iostream>
#include <thread>

#include <tobas_aso_core/dshot.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    cerr << "Usage: " << argv[0] << " <Gain> <Target RPM>" << endl;
    return EXIT_FAILURE;
  }
  const uint8_t gain = stoi(argv[1]);
  const uint16_t tar_rpm = stoi(argv[2]);

  aso::DShot dshot;

  if (!dshot.initialize())
    throw runtime_error("Failed to initialize DShot driver.");

  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
    if (!dshot.setKv(ch, 920 * (M_PI / 30)))
      throw runtime_error("Failed to set Kv.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send Kv.");
  this_thread::sleep_for(1ms);

  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
    if (!dshot.setInternalResistance(ch, 0.25))
      throw runtime_error("Failed to set internal resistance.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send internal resistance.");
  this_thread::sleep_for(1ms);

  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
    if (!dshot.setPropellerDiameter(ch, 9 * 0.0254))
      throw runtime_error("Failed to set propeller diameter.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send propeller diameter.");
  this_thread::sleep_for(1ms);

  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
    if (!dshot.setMomentConstant(ch, 5.442e-5))
      throw runtime_error("Failed to set moment constant.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send moment constant.");
  this_thread::sleep_for(1ms);

  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
    if (!dshot.setNumPoles(ch, 14))
      throw runtime_error("Failed to set the number of poles.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send the number of poles.");
  this_thread::sleep_for(1ms);

  for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
    if (!dshot.setSpeedControlGain(ch, gain))
      throw runtime_error("Failed to set the speed control gain.");
  if (!dshot.transfer())
    throw runtime_error("Failed to send the speed control gain.");
  this_thread::sleep_for(1ms);

  while (true)
  {
    for (size_t ch = 0; ch < aso::DShot::kChannelSize; ++ch)
      if (!dshot.setTargetSpeed(ch, tar_rpm * (M_PI / 30)))
        throw runtime_error("Failed to set target speed.");

    if (!dshot.transfer())
      throw runtime_error("Failed to send target speed.");

    dshot.printCurrentStates();
    cout << "----------" << endl;

    this_thread::sleep_for(10ms);
  }
}
