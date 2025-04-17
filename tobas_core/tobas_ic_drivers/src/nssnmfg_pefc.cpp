#include <iostream>

#include "../include/tobas_ic_drivers/nssnmfg_pefc.hpp"

#define TIMEOUT_MS 1000

using namespace std;

namespace driver
{
NssnmfgPEFC::NssnmfgPEFC(std::function<void(const Packet&)> packet_cb) : packet_cb_(packet_cb)
{
}

bool NssnmfgPEFC::initialize(const char* uart_device)
{
  if (!uart_.initialize(uart_device, true))
    return false;

  if (!uart_.setBaudRate(38400))
    return false;

  if (!uart_.setDataBits(8))
    return false;

  if (!uart_.setSingleStopBit())
    return false;

  if (!uart_.disableParity())
    return false;

  if (!uart_.setTimeout(TIMEOUT_MS / 100))
    return false;

  return true;
}

void NssnmfgPEFC::start()
{
  read_thread_ = thread(bind(&NssnmfgPEFC::readThreadFunc, this));
}

void NssnmfgPEFC::spin()
{
  read_thread_.join();
}

void NssnmfgPEFC::readThreadFunc()
{
  while (true)
  {
    // Check header
    if (!uart_.receive(buf_ + kHeaderIdx, 1))
      continue;
    if (buf_[kHeaderIdx] != 0xAA)
      continue;

    // Receive packet
    for (size_t i = kHeaderIdx + 1; i < kPacketSize; ++i)
      if (!uart_.receive(buf_ + i, 1))
        continue;

    // CheckSum
    if (!checkSum())
      continue;

    // Decode packet
    decode();

    // Call user callback
    packet_cb_(packet_);
  }
}

bool NssnmfgPEFC::checkSum() const
{
  uint8_t checksum = 0;
  for (size_t i = 0; i < kCheckSumIdx; ++i)
    checksum ^= buf_[i];

  if (checksum != buf_[kCheckSumIdx])
  {
    cerr << "CheckSum failed: " << checksum << " != " << buf_[kCheckSumIdx] << endl;
    return false;
  }

  return true;
}

void NssnmfgPEFC::decode()
{
  // Error
  const auto error = read2Bytes(kErrorIdx);
  packet_.error.stack_temperature_abnormal = (error >> 6) & 1;
  packet_.error.dcdc_current_too_high = (error >> 7) & 1;
  packet_.error.dcdc_voltage_too_low = (error >> 8) & 1;
  packet_.error.fc_current_too_high = (error >> 9) & 1;
  packet_.error.fc_current_too_low = (error >> 10) & 1;
  packet_.error.stack_leak = (error >> 11) & 1;
  packet_.error.battery_voltage_abnormal = (error >> 12) & 1;
  packet_.error.hydrogen_level_too_low = (error >> 13) & 1;

  // Stack
  packet_.stack.voltage = read2Bytes(kStackVoltageIdx);
  packet_.stack.current = read2Bytes(kStackCurrentIdx);
  packet_.stack.power = read2Bytes(kStackPowerIdx);
  packet_.stack.pressure = read2Bytes(kStackPressureIdx);
  packet_.stack.temperature = read2Bytes(kStackTemperatureIdx);

  // Battery
  packet_.battery.voltage = read2Bytes(kBatteryVoltageIdx);
  packet_.battery.current = read2Bytes(kBatteryCurrentIdx);
  packet_.battery.power = read2Bytes(kBatteryPowerIdx);

  // Tank
  packet_.tank.pressure = read2Bytes(kTankPressureIdx);
}
}  // namespace driver
