#include <iostream>
#include <boost/multiprecision/cpp_int.hpp>

#include "../include/tobas_aso_core/sbus.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;
using namespace boost::multiprecision;

namespace aso
{
SBUS::SBUS()
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

  // TODO: SBUSは偶数パリティのはずだが，パリティチェックを有効にすると1バイトも取得できない．
  if (!uart_dev_.disableParity())
    return false;

  if (!uart_dev_.setTimeout(1))  // S.BUSは50 ~ 100Hzだから，インターバルの最大値は20ms
    return false;

  return true;
}

bool SBUS::update()
{
  if (!read())
    return false;

  decodeData();
  decodeFlags();

  return true;
}

bool SBUS::read()
{
  // SBUSのパケットを取得
  // 一定時間のLOWでブレークポイントとみなされるため，25バイトを取得したときそれが複数のパケットにまたがっていることはない．
  const auto read_size = ::read(uart_dev_.fd(), packet_.data(), kPacketSize);

  // 読み取ったデータサイズに応じて場合分け
  switch (read_size)
  {
    case kPacketSize:
    {
      // Check start byte
      const auto& start_byte = packet_.at(kStartIdx);
      if (start_byte != 0x0F)
      {
        cerr << "Invalid start byte: " << hex << uppercase << (int)start_byte << endl;
        return false;
      }

      // Check end byte
      const auto& end_byte = packet_.at(kEndIdx);
      switch (end_byte)
      {
        case 0x00:  // SBUS1
          break;
        case 0x04:  // SBUS2 telemetry slots 0-7
          break;
        case 0x14:  // SBUS2 telemetry slots 8-15
          break;
        case 0x24:  // SBUS2 telemetry slots 16-23
          break;
        case 0x34:  // SBUS2 telemetry slots 24-31
          break;
        default:
          cerr << "Invalid end byte: " << hex << uppercase << (int)end_byte << endl;
          return false;
      }

      break;
    }

    case kTelemSize:
      // TODO
      break;

    default:
      cerr << "Invalid packet size: " << read_size << endl;
      return false;
  }

  return true;
}

void SBUS::decodeData()
{
  // 繰り上がりが面倒なので，一旦データを1つのビット列に変換する．
  uint256_t data = 0;
  for (size_t idx = 0; idx < kDataSize; ++idx)
    data |= (static_cast<uint256_t>(packet_.at(kDataIdx + idx)) << (kDataBits * idx));

  // 11ビットずつ取り出す
  constexpr uint16_t kMask = (1 << kChannelBits) - 1;
  for (size_t ch = 0; ch < kChannelSize; ++ch)
    out_.periods.at(ch) = ((data >> (kChannelBits * ch)) & kMask).convert_to<uint16_t>();
}

void SBUS::decodeFlags()
{
  const auto& flags_byte = packet_.at(kFlagsIdx);
  out_.ch17 = (flags_byte >> 0) & 1;
  out_.ch18 = (flags_byte >> 1) & 1;
  out_.frame_lost = (flags_byte >> 2) & 1;
  out_.failsave_activated = (flags_byte >> 3) & 1;
}
}  // namespace aso
