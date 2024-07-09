#include <boost/multiprecision/cpp_int.hpp>

#include "../include/tobas_a1_core/sbus.hpp"

using namespace boost::multiprecision;

namespace a1
{
SBUS::SBUS()
{
}

bool SBUS::initialize()
{
  if (!uart_dev_.initialize(kUartDev, true))
    return false;

  if (!uart_dev_.setBaudRate(kBaudRate))
    return false;

  if (!uart_dev_.setDataBits(kDataBits))
    return false;

  if (!uart_dev_.setDoubleStopBit())
    return false;

  if (!uart_dev_.enableParity(linux::UARTdev::PARITY_EVEN))
    return false;

  if (!uart_dev_.setMinimumChars(1))
    return false;

  return true;
}

bool SBUS::update()
{
  if (!read())
    return false;

  decode();

  return true;
}

bool SBUS::read()
{
  uint8_t byte = 0x00;

  // Wait for start byte
  while (byte != 0x0F)
    if (!uart_dev_.receive(&byte, 1))
      return false;

  // Get 22 bytes
  if (!uart_dev_.receive(data_, kDataSize))
    return false;

  // Skip flags
  if (!uart_dev_.receive(&byte, 1))
    return false;

  // Check end byte
  if (!uart_dev_.receive(&byte, 1))
    return false;
  if (byte != 0x00)
    return false;

  return true;
}

void SBUS::decode()
{
  // 繰り上がりが面倒なので，一旦データを1つのビット列に変換する．
  uint256_t data = 0;
  for (uint8_t idx = 0; idx < kDataSize; ++idx)
    data |= (static_cast<uint256_t>(data_[idx]) << (kDataBits * idx));

  // 11ビットずつデコード
  constexpr uint16_t mask = (1 << kChannelBits) - 1;
  for (uint8_t ch = 0; ch < kChannelSize; ++ch)
    periods_[ch] = ((data >> (kChannelBits * ch)) & mask).convert_to<uint16_t>();
}
}  // namespace a1
