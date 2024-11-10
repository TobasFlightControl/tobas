#include <iostream>
#include <thread>
#include <set>
#include <boost/multiprecision/cpp_int.hpp>

#include <tobas_std_tools/console.hpp>

#include "../include/tobas_ic_drivers/sbus.hpp"

#define TIMEOUT_MS 1000
#define TIMEOUT_ERROR_MSG "Failed to receive SBUS byte in 1 second."

using namespace std;
using namespace boost::multiprecision;

namespace driver
{
SBUS::SBUS(function<void(const Packet&)> packet_cb) : packet_cb_(packet_cb)
{
}

bool SBUS::initialize(const char* device)
{
  if (!uart_.initialize(device, true))
    return false;

  if (!uart_.setBaudRate(kBaudRate))
    return false;

  if (!uart_.setDataBits(kDataBits))
    return false;

  if (!uart_.setDoubleStopBit())
    return false;

  if (!uart_.enableParity(linux::UARTdev::PARITY_EVEN))
    return false;

  if (!uart_.setTimeout(TIMEOUT_MS / 100))
    return false;

  return true;
}

void SBUS::spin()
{
  read_thread_.join();
}

void SBUS::start()
{
  read_thread_ = thread(bind(&SBUS::readThreadFunc, this));
}

void SBUS::readThreadFunc()
{
  const set<uint8_t> end_bytes{ 0x00, 0x04, 0x14, 0x24, 0x34 };

  uint8_t start_byte, end_byte, flags;
  array<uint8_t, kDataSize> data;

  while (true)
  {
    // インバータが悪いのかLinuxのUARTデバイスにデータが勝手に分割されるため，一括ではなく1バイトずつ取得する．
    // FIXME: SBUSドライバの起動時に偶然スタートバイトでない0x0Fが先頭にきているとバグるはず

    // Start byte
    if (!uart_.receive(&start_byte, 1))
    {
      cerr << TIMEOUT_ERROR_MSG << endl;
      continue;
    }
    PRINT_DEBUG("Start byte: " << hex << uppercase << (int)start_byte << dec << nouppercase);
    if (start_byte != 0x0F)
      continue;

    // Data
    for (size_t i = 0; i < kDataSize; ++i)
    {
      if (!uart_.receive(&data[i], 1))
      {
        cerr << TIMEOUT_ERROR_MSG << endl;
        continue;
      }
      PRINT_DEBUG("Data byte " << i + 1 << ": " << hex << uppercase << (int)data[i] << dec << nouppercase);
    }

    // Flags
    if (!uart_.receive(&flags, 1))
    {
      cerr << TIMEOUT_ERROR_MSG << endl;
      continue;
    }
    PRINT_DEBUG("Flags byte: " << hex << uppercase << (int)flags << dec << nouppercase);

    // End byte
    if (!uart_.receive(&end_byte, 1))
    {
      cerr << TIMEOUT_ERROR_MSG << endl;
      continue;
    }
    PRINT_DEBUG("End byte: " << hex << uppercase << (int)end_byte << dec << nouppercase);
    if (!end_bytes.contains(end_byte))
    {
      cerr << "Invalid end byte: " << hex << uppercase << (int)end_byte << dec << nouppercase << endl;
      continue;
    }

    // Decode packet
    decodeData(data);
    decodeFlags(flags);

    // Call user callback
    packet_cb_(packet_);
  }
}

void SBUS::decodeData(const array<uint8_t, kDataSize>& data)
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
}  // namespace driver
