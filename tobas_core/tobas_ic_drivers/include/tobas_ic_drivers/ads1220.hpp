#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace driver
{
/**
 * @brief A linux driver of 2-channel ADC.
 *
 * Datasheet: https://www.ti.com/jp/lit/ds/symlink/ads1220.pdf
 */
class ADS1220
{
  static constexpr size_t kSPIBufSize = 3;
  static constexpr uint32_t kSPIClockFreq = 5'000'000;  // Minimum period is 150ns (6.67MHz)
  static constexpr double kVref = 2.048;                // Internal 2.048-V reference (p.42)
  static constexpr double kGain = 1.;

public:
  explicit ADS1220();

  bool initialize(const char* spi_device);

  /* Read the current value of the voltage channel [-1, 1] */
  bool readVoltage(double& dst);

  /* Read the current value of the current channel [-1, 1] */
  bool readCurrent(double& dst);

private:
  /* 8.5.3: Commands */
  enum command_t : uint8_t
  {
    RESET = 0b00000110,      // Reset the device
    START = 0b00001000,      // Start of restart conversions
    POWERDOWN = 0b00000010,  // Enter power-down mode
    RDATA = 0b00010000,      // Read data by command
    RREG = 0b00100000,       // Read nn registers starting at address rr
    WREG = 0b01000000,       // Write nn registers starting at address rr

    DUMMY = 0xFF,
  };

  /* 8.6: Register Map (p.39) */
  enum register_t : uint8_t
  {
    CFG_REG_0 = 0b00 << 2,  // Configuration Register 0
    CFG_REG_1 = 0b01 << 2,  // Configuration Register 1
    CFG_REG_2 = 0b10 << 2,  // Configuration Register 2
    CFG_REG_3 = 0b11 << 2,  // Configuration Register 3
  };

  enum config_t : uint8_t
  {
    // Configuration Register 0
    MUX_AIN0_AVSS = 0b1000 << 4,
    MUX_AIN1_AVSS = 0b1001 << 4,
    MUX_AIN2_AVSS = 0b1010 << 4,
    MUX_AIN3_AVSS = 0b1011 << 4,
    GAIN_1 = 0b000 << 1,
    GAIN_2 = 0b001 << 1,
    GAIN_4 = 0b010 << 1,
    GAIN_8 = 0b011 << 1,
    GAIN_16 = 0b100 << 1,
    GAIN_32 = 0b101 << 1,
    GAIN_64 = 0b110 << 1,
    GAIN_128 = 0b111 << 1,
    PGA_DISABLED = 1 << 0,

    // Configuration Register 1
    DR_20SPS = 0b000 << 5,
    DR_45SPS = 0b001 << 5,
    DR_90SPS = 0b010 << 5,
    DR_175SPS = 0b011 << 5,
    DR_330SPS = 0b100 << 5,
    DR_600SPS = 0b101 << 5,
    DR_1000SPS = 0b110 << 5,
    MODE_NORMAL = 0b00 << 3,
    MODE_DUTY_CYCLE = 0b01 << 3,
    MODE_TURBO = 0b10 << 3,
    CM_CONTINUOUS = 1 << 2,
    TS_ENABLE = 1 << 1,
    BCS_ON = 1 << 0,

    // Configuration Register 2
    // TODO

    // Configuration Register 3
    // TODO
  };

  linux::SPIdev spi_;
  uint8_t tx_buf_[kSPIBufSize];
  uint8_t rx_buf_[kSPIBufSize];

  bool reset();
  bool start();
  bool powerDown();
  bool sendStandAloneCommand(const uint8_t& cmd);
  bool configure(const uint8_t& rr, const uint8_t& tar_cfg);
};
}  // namespace driver
