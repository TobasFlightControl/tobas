#pragma once

#include <cinttypes>

namespace a1
{
static constexpr char kRasPiI2CDev[] = "/dev/i2c-1";

namespace uart_device
{
static constexpr char kSbusDev[] = "/dev/serial0";
}

namespace spi_device
{
static constexpr char kAdcDev[] = "/dev/spidev?.?";    // TODO
static constexpr char kImuDev[] = "/dev/spidev?.?";    // TODO
static constexpr char kGnssDev[] = "/dev/spidev?.?";   // TODO
static constexpr char kDshotDev[] = "/dev/spidev?.?";  // TODO
static constexpr char kPwmDev[] = "/dev/spidev?.?";    // TODO
}  // namespace spi_device

namespace i2c_address
{
static constexpr uint8_t kMagAddress = 0x3C;
static constexpr uint8_t kBaroAddress = 0x5C;
}  // namespace i2c_address
}  // namespace a1
