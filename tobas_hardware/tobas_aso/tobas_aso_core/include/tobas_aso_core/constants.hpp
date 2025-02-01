#pragma once

#include <cstdint>

namespace aso
{
static constexpr char kRasPiI2CDev[] = "/dev/i2c-1";

namespace uart_device
{
static constexpr char kSbusDev[] = "/dev/ttyAMA0";
}

namespace spi_device
{
static constexpr char kImuDev[] = "/dev/spidev0.0";
static constexpr char kBatteryDev[] = "/dev/spidev0.1";
static constexpr char kDshotDev[] = "/dev/spidev1.0";
static constexpr char kPwmDev[] = "/dev/spidev1.1";
static constexpr char kGnssDev[] = "/dev/spidev1.2";
}  // namespace spi_device

namespace i2c_address
{
static constexpr uint8_t kMagAddress = 0b0011110;
static constexpr uint8_t kBaroAddress = 0b1011100;
}  // namespace i2c_address
}  // namespace aso
