#include <iostream>
#include <thread>
#include <boost/multiprecision/cpp_int.hpp>

#include "../include/tobas_aso_core/sbus.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;
using namespace boost::multiprecision;

namespace aso
{
SBUS::SBUS(std::function<void(const Packet&)> packet_cb) : packet_cb_(packet_cb)
{
}

bool SBUS::initialize()
{
  if (!uart_dev_.initialize(uart_device::kSbusDev, true))
    return false;

  if (!uart_dev_.setNonStandardBaudRate(kBaudRate))
    return false;

  if (!uart_dev_.setDataBits(kDataBits))
    return false;

  if (!uart_dev_.setDoubleStopBit())
    return false;

  if (!uart_dev_.enableParity(linux::UARTdev::PARITY_EVEN))
    return false;

  // 信号読み取りを開始
  read_thread_ = thread(bind(&SBUS::readThreadFunc, this));

  return true;
}

void SBUS::spin()
{
  read_thread_.join();
}

void SBUS::readThreadFunc()
{
  const std::set<uint8_t> end_bytes{ 0x00, 0x04, 0x14, 0x24, 0x34 };

  while (true)
  {
    // インバータが悪いのかLinuxのUARTデバイスにデータが勝手に分割されるため，一括ではなく1バイトずつ取得する．
    // FIXME: SBUSドライバの起動時に偶然スタートバイトでない0x0Fが先頭にきているとバグるはず

    // Start byte
    if (readByte() != 0x0F)
      continue;

    // Data
    for (size_t i = 0; i < kDataSize; ++i)
      data_[i] = readByte();

    // Flags
    const auto flags = readByte();

    // End byte
    const auto end_byte = readByte();
    if (!end_bytes.contains(end_byte))
    {
      cerr << "Invalid end byte: " << hex << uppercase << (int)end_byte << endl;
      continue;
    }

    // Decode packet
    decodeData(data_);
    decodeFlags(flags);

    // Call user callback
    packet_cb_(packet_);
  }
}

uint8_t SBUS::readByte()
{
  if (::read(uart_dev_.fd(), &byte_, 1) != 1)
    throw runtime_error("Failed to read 1 byte.");
  return byte_;
}

void SBUS::decodeData(const std::array<uint8_t, kDataSize>& data)
{
  // 繰り上がりが面倒なので，一旦データを1つのビット列に変換する．
  uint256_t bits = 0;
  for (size_t idx = 0; idx < kDataSize; ++idx)
    bits |= (static_cast<uint256_t>(data.at(idx)) << (kDataBits * idx));

  // 11ビットずつ取り出す
  constexpr uint16_t kMask = (1 << kChannelBits) - 1;
  for (size_t ch = 0; ch < kChannelSize; ++ch)
    packet_.periods.at(ch) = ((bits >> (kChannelBits * ch)) & kMask).convert_to<uint16_t>();
}

void SBUS::decodeFlags(uint8_t flags)
{
  packet_.ch17 = (flags >> 0) & 1;
  packet_.ch18 = (flags >> 1) & 1;
  packet_.frame_lost = (flags >> 2) & 1;
  packet_.failsave_activated = (flags >> 3) & 1;
}
}  // namespace aso
