#pragma once

#include <thread>
#include <functional>

#include <tobas_linux/uart_dev.hpp>

namespace driver
{
class NssnmfgPEFC
{
  static constexpr size_t kHeaderIdx = 0;
  static constexpr size_t kErrorIdx = kHeaderIdx + 1;
  static constexpr size_t kStackVoltageIdx = kErrorIdx + 2;
  static constexpr size_t kStackCurrentIdx = kStackVoltageIdx + 2;
  static constexpr size_t kStackPowerIdx = kStackCurrentIdx + 2;
  static constexpr size_t kStackPressureIdx = kStackPowerIdx + 2;
  static constexpr size_t kStackTemperatureIdx = kStackPressureIdx + 2;
  static constexpr size_t kBatteryVoltageIdx = kStackTemperatureIdx + 2;
  static constexpr size_t kBatteryCurrentIdx = kBatteryVoltageIdx + 2;
  static constexpr size_t kBatteryPowerIdx = kBatteryCurrentIdx + 2;
  static constexpr size_t kTankPressureIdx = kBatteryPowerIdx + 2;
  static constexpr size_t kCheckSumIdx = kTankPressureIdx + 2;
  static constexpr size_t kPacketSize = kCheckSumIdx + 1;

public:
  struct Packet
  {
    // Error
    struct Error
    {
      bool stack_temperature_abnormal;
      bool dcdc_current_too_high;
      bool dcdc_voltage_too_low;
      bool fc_current_too_high;
      bool fc_current_too_low;
      bool stack_leak;
      bool battery_voltage_abnormal;
      bool hydrogen_level_too_low;
    } error;

    // Stack
    struct Stack
    {
      uint16_t voltage;      // [mV]
      uint16_t current;      // [mA]
      uint16_t power;        // [W]
      uint16_t pressure;     // [KPa]
      uint16_t temperature;  // [degC]
    } stack;

    // Battery
    struct Battery
    {
      uint16_t voltage;  // [mV]
      uint16_t current;  // [mA]
      uint16_t power;    // [W]
    } battery;

    // Tank
    struct Tank
    {
      uint16_t pressure;  // [KPa]
    } tank;
  };

  explicit NssnmfgPEFC(std::function<void(const Packet&)> packet_cb);

  bool initialize(const char* uart_device);
  void start();
  void spin();

private:
  const std::function<void(const Packet&)> packet_cb_;

  linux::UARTdev uart_;
  uint8_t buf_[kPacketSize];
  Packet packet_;

  std::thread read_thread_;
  void readThreadFunc();

  bool checkSum();
  void decode();
};
}  // namespace driver
