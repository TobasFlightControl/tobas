#pragma once

#include <array>

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/spi_dev.hpp>

namespace aso
{
/* cf. https://betaflight.com/docs/development/api/dshot */
class DShot
{
public:
  static constexpr size_t kChannelSize = 8;
  static constexpr size_t kSPIBufSize = kChannelSize + 1;  // Data + CRC32

  enum command_t : uint16_t
  {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEEP1 = 1,
    DSHOT_CMD_BEEP2 = 2,
    DSHOT_CMD_BEEP3 = 3,
    DSHOT_CMD_BEEP4 = 4,
    DSHOT_CMD_BEEP5 = 5,
    DSHOT_CMD_ESC_INFO = 6,
    DSHOT_CMD_SPIN_DIRECTION_1 = 7,
    DSHOT_CMD_SPIN_DIRECTION_2 = 8,
    DSHOT_CMD_3D_MODE_OFF = 9,
    DSHOT_CMD_3D_MODE_ON = 10,
    DSHOT_CMD_SETTINGS_REQUEST = 11,
    DSHOT_CMD_SAVE_SETTINGS = 12,
    DSHOT_EXTENDED_TELEMETRY_ENABLE = 13,
    DSHOT_EXTENDED_TELEMETRY_DISABLE = 14,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON = 22,
    DSHOT_CMD_LED1_ON = 23,
    DSHOT_CMD_LED2_ON = 24,
    DSHOT_CMD_LED3_ON = 25,
    DSHOT_CMD_LED0_OFF = 26,
    DSHOT_CMD_LED1_OFF = 27,
    DSHOT_CMD_LED2_OFF = 28,
    DSHOT_CMD_LED3_OFF = 29,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE = 33,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 34,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY = 35,
    DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY = 42,
    DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY = 43,
    DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY = 44,
    DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY = 45,
    DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY = 46,
    DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY = 47,
  };

private:
  // Commands
  static constexpr uint8_t kSetThrottleCmd = 0;
  static constexpr uint8_t kSetTargetRPMCmd = 1;
  static constexpr uint8_t kSetKvCmd = 2;
  static constexpr uint8_t kSetResistanceCmd = 3;
  static constexpr uint8_t kSetDiameterCmd = 4;
  static constexpr uint8_t kSetMomentConstCmd = 5;
  static constexpr uint8_t kSetHalfNumPolesCmd = 6;
  static constexpr uint8_t kSetGainCmd = 7;

  static constexpr size_t kChannelBytes = 4;                           // 1チャネルあたりのバイト数
  static constexpr size_t kSpiBufSize = kChannelSize * kChannelBytes;  // SPIバッファのサイズ
  static constexpr uint32_t kSPIClockFreq = 30'000'000;                // [Hz]

public:
  explicit DShot();

  bool initialize();
  bool transfer();

  /* Set the DShot throttle directory. */
  bool setThrottle(size_t ch, uint16_t throttle);
  /* Set the target motor rotating speed [rad/s] */
  bool setTargetSpeed(size_t ch, double rps);
  /* Set the KV value [rad/s/V] */
  bool setKv(size_t ch, double kv_si);
  /* Set the internal resistance [Ω] */
  bool setInternalResistance(size_t ch, double resistance);
  /* Set the propeller diameter [m] */
  bool setPropellerDiameter(size_t ch, double diameter);
  /* Set the moment constant scaled by the propeller diameter [Nm/(rad/s)^2/m^4] */
  bool setMomentConstant(size_t ch, double moment_const);
  /* Set the number of motor poles */
  bool setNumPoles(size_t ch, uint16_t num_poles);
  /* Set the motor speed control gain (2 to the x-1 power). No feedback when 0 is specified. */
  bool setSpeedControlGain(size_t ch, uint8_t gain);

  /* Get the validity of the telemetry */
  bool getValidity(size_t ch);
  /* Get the current motor rotating speed [rad/s] */
  double getSpeed(size_t ch);
  /* Get the current ESC temperature [degC] */
  double getTemperature(size_t ch);
  /* Get the current ESC input voltage [V] */
  double getVoltage(size_t ch);
  /* Get the current ESC current [A] */
  double getCurrent(size_t ch);

  void printCurrentState(size_t ch);
  void printCurrentStates();

private:
  linux::SPIdev spi_;
  uint32_t tx_buf_[kSPIBufSize];
  uint32_t rx_buf_[kSPIBufSize];

  std::array<uint16_t, kChannelSize> half_num_poles_;

  algo::CRC32Left crc_;

  bool checkChannelSize(size_t ch);
};
}  // namespace aso
