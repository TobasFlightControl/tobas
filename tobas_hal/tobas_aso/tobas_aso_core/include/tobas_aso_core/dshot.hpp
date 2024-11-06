#pragma once

#include <tobas_linux/spi_dev.hpp>

namespace aso
{
/* cf. https://betaflight.com/docs/development/api/dshot */
class DShot
{
public:
  static constexpr size_t kChannelSize = 8;

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
  static constexpr uint32_t kSpiClockFreq = 45'000'000;  // [Hz] 最大は50MHzだが，高すぎるとMISOが失敗する．

public:
  explicit DShot();

  bool initialize();
  bool transfer();

  bool setThrottle(size_t ch, uint16_t throttle);
  bool setTargetSpeed(size_t ch, double rps);
  bool setKv(size_t ch, double kv_si);
  bool setInternalResistance(size_t ch, double resistance);
  bool setPropellerDiameter(size_t ch, double diameter);
  bool setMomentConstant(size_t ch, double moment_const);
  bool setNumPoles(size_t ch, uint32_t num_poles);
  bool setSpeedControlGain(size_t ch, uint32_t gain);

  bool isValid(size_t ch);
  double getCurrentSpeed(size_t ch);

private:
  linux::SPIdev spi_;

  bool checkChannelSize(size_t ch);
};
}  // namespace aso
