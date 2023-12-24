#include <stdexcept>
#include <cstring>
#include <cassert>
#include <chrono>

#include "../../include/Common/Ublox.h"
#include "../../include/Common/Util.h"

#define GPS_DEVICE "/dev/spidev0.0"

using namespace std;
using namespace chrono;

UBXScanner::UBXScanner()
{
  reset();
}

void UBXScanner::reset()
{
  message_length_ = 0;
  position_ = 0;
  state_ = Sync1;
}

int UBXScanner::update(const uint8_t& data)
{
  if (state_ != Done)
    message_[position_++] = data;

  switch (state_)
  {
    case Sync1:
      if (data == 0xb5)
        state_ = Sync2;
      else
        reset();
      break;

    case Sync2:
      if (data == 0x62)
        state_ = Class;
      else if (data == 0xb5)
        state_ = Sync1;
      else
        reset();
      break;

    case Class:
      state_ = ID;
      break;

    case ID:
      state_ = Length1;
      break;

    case Length1:
      payload_length_ = data;
      state_ = Length2;
      break;

    case Length2:
      payload_length_ += data << 8;
      state_ = Payload;
      break;

    case Payload:
      if (position_ == payload_length_ + 6)
        state_ = CK_A;
      if (position_ >= 1022)
        reset();
      break;

    case CK_A:
      state_ = CK_B;
      break;

    case CK_B:
      message_length_ = 6 + payload_length_ + 2;
      state_ = Done;
      break;

    case Done:
      break;

    default:
      break;
  }

  return state_;
}

UBXParser::UBXParser(UBXScanner* ubxsc) : scanner_(ubxsc), message_(ubxsc->getMessage())
{
}

uint16_t UBXParser::calcId()
{
  const auto& position = scanner_->getPosition();
  const auto& length = scanner_->getMessageLength();
  const auto pos = position - length;  // count the message start position
  const auto s = message_ + pos;

  // All UBX messages start with 2 sync chars: 0xb5 and 0x62
  if (*s != 0xb5)
    return 0;
  if (*(s + 1) != 0x62)
    return 0;

  // Count the checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (uint32_t i = 2; i + 2 < length; ++i)  // 符号なしの引き算はオーバーフローのリスクがある
  {
    CK_A += *(s + i);
    CK_B += CK_A;
  }
  if (CK_A != *(s + length - 2))
    return 0;
  if (CK_B != *(s + length - 1))
    return 0;

  // If we got everything right, then it's time to decide, what type of message this is
  // ID is a two-byte number with little endianness
  return latest_id_ = (*(s + 2)) << 8 | (*(s + 3));
}

Ublox::Ublox()
  : spi_dev_(GPS_DEVICE, kSpiSpeedHz), scanner_(new UBXScanner()), parser_(new UBXParser(scanner_))
{
  if (!enableMsg(ACK_NAK, true) || !enableMsg(ACK_ACK, true))
  {
    throw runtime_error("Failed to configure.");
  }
}

Ublox::Ublox(UBXScanner* scan, UBXParser* pars)
  : spi_dev_(GPS_DEVICE, kSpiSpeedHz), scanner_(scan), parser_(pars)
{
  if (!enableMsg(ACK_NAK, true) || !enableMsg(ACK_ACK, true))
  {
    throw runtime_error("Failed to configure.");
  }
}

bool Ublox::enableMsg(message_t msg, bool enable)
{
  CfgMsg cfg_msg;

  cfg_msg.msgClass = msg >> 8;                       // 上位8ビット
  cfg_msg.msgID = static_cast<uint8_t>(msg & 0xFF);  // 下位8ビット
  cfg_msg.rate = enable;

  return sendMessage(CLASS_CFG, ID_CFG_MSG, &cfg_msg, sizeof(CfgMsg));
}

bool Ublox::enableAllMsgs(bool enable)
{
  bool ok = true;

  ok &= enableMsg(NAV_POSLLH, enable);
  ok &= enableMsg(NAV_STATUS, enable);
  ok &= enableMsg(NAV_DOP, enable);
  ok &= enableMsg(NAV_PVT, enable);
  ok &= enableMsg(NAV_VELNED, enable);
  ok &= enableMsg(NAV_TIMEGPS, enable);
  ok &= enableMsg(NAV_TIMEUTC, enable);
  ok &= enableMsg(NAV_COV, enable);

  ok &= enableMsg(MON_HW, enable);
  ok &= enableMsg(MON_HW2, enable);

  return ok;
}

bool Ublox::configureSolutionRate(uint16_t meas_rate, uint16_t nav_rate, uint16_t time_ref)
{
  CfgRate cfg_rate;

  cfg_rate.measRate = meas_rate;
  cfg_rate.navRate = nav_rate;
  cfg_rate.timeRef = time_ref;

  return sendMessage(CLASS_CFG, ID_CFG_RATE, &cfg_rate, sizeof(CfgRate));
}

bool Ublox::configureDynamicsModel(dynamics_model dyn_model)
{
  CfgNav5 cfg_nav5;
  memset(&cfg_nav5, 0, sizeof(CfgNav5));

  cfg_nav5.mask = 1;  // dynModelのみ更新
  cfg_nav5.dynModel = dyn_model;

  return sendMessage(CLASS_CFG, ID_CFG_NAV5, &cfg_nav5, sizeof(CfgNav5));
}

bool Ublox::configureGnss_GPS(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  assert(max_track_ch >= kMinMaxTrkChForMajorGnss);
  return configureGnss(GPS, res_track_ch, max_track_ch, enable);
}

bool Ublox::configureGnss_SBAS(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  return configureGnss(SBAS, res_track_ch, max_track_ch, enable);
}

bool Ublox::configureGnss_Galileo(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  assert(max_track_ch >= kMinMaxTrkChForMajorGnss);
  return configureGnss(GALILEO, res_track_ch, max_track_ch, enable);
}

bool Ublox::configureGnss_BeiDou(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  assert(max_track_ch >= kMinMaxTrkChForMajorGnss);
  return configureGnss(BEIDOU, res_track_ch, max_track_ch, enable);
}

bool Ublox::configureGnss_QZSS(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  return configureGnss(QZSS, res_track_ch, max_track_ch, enable);
}

bool Ublox::configureGnss_GLONASS(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  assert(max_track_ch >= kMinMaxTrkChForMajorGnss);
  return configureGnss(GLONASS, res_track_ch, max_track_ch, enable);
}

uint16_t Ublox::update()
{
  int status = -1;
  uint8_t to_gps_data = 0, from_gps_data = 0;

  while (status != UBXScanner::Done)
  {
    // From now on, we will send zeroes to the receiver, which it will ignore
    // However, we are simultaneously getting useful information from it
    spi_dev_.transfer(&to_gps_data, &from_gps_data, 1);

    // Scanner checks the message structure with every byte received
    status = scanner_->update(from_gps_data);
  }

  scanner_->reset();
  return parser_->calcId();
}

void Ublox::decode(NavPosllhPayload& data) const
{
  if (parser_->getLatestMsg() != Ublox::NAV_POSLLH)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.lon = int((*(s + 13) << 24) | (*(s + 12) << 16) | (*(s + 11) << 8) | (*(s + 10))) * 1e-7;
  data.lat = int((*(s + 17) << 24) | (*(s + 16) << 16) | (*(s + 15) << 8) | (*(s + 14))) * 1e-7;
  data.hMSL = int((*(s + 25) << 24) | (*(s + 24) << 16) | (*(s + 23) << 8) | (*(s + 22))) * 1e-3;
}

void Ublox::decode(NavStatusPayload& data) const
{
  if (parser_->getLatestMsg() != Ublox::NAV_STATUS)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.gpsFix = uint8_t(*(s + 10));

  const auto flags = uint8_t(*(s + 11));
  data.gpsFixOk = (flags >> 0) & 1;
}

void Ublox::decode(NavDopPayload&) const
{
  throw;  // TODO
}

void Ublox::decode(NavPvtPayload& data) const
{
  if (parser_->getLatestMsg() != Ublox::NAV_PVT)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.year = uint16_t((*(s + 11) << 8) | (*(s + 10)));
  data.month = uint8_t(*(s + 12));
  data.day = uint8_t(*(s + 13));
  data.hour = uint8_t(*(s + 14));
  data.min = uint8_t(*(s + 15));
  data.sec = uint8_t(*(s + 16));

  const auto valid = uint8_t(*(s + 17));
  data.validDate = (valid >> 0) & 1;
  data.validTime = (valid >> 1) & 1;
  data.fullyResolved = (valid >> 2) & 1;
  data.validMag = (valid >> 3) & 1;

  data.tAcc = uint32_t((*(s + 21) << 24) | (*(s + 20) << 16) | (*(s + 19) << 8) | (*(s + 18)));
  data.nano = int((*(s + 25) << 24) | (*(s + 24) << 16) | (*(s + 23) << 8) | (*(s + 22)));

  data.fixType = uint8_t(*(s + 26));

  const auto flags = uint8_t(*(s + 27));
  data.gnssFixOk = (flags >> 0) & 1;

  data.lon = int((*(s + 33) << 24) | (*(s + 32) << 16) | (*(s + 31) << 8) | (*(s + 30))) * 1e-7;
  data.lat = int((*(s + 37) << 24) | (*(s + 36) << 16) | (*(s + 35) << 8) | (*(s + 34))) * 1e-7;
  data.hMSL = int((*(s + 45) << 24) | (*(s + 44) << 16) | (*(s + 43) << 8) | (*(s + 42))) * 1e-3;
  data.velN = int((*(s + 57) << 24) | (*(s + 56) << 16) | (*(s + 55) << 8) | (*(s + 54))) * 1e-3;
  data.velE = int((*(s + 61) << 24) | (*(s + 60) << 16) | (*(s + 59) << 8) | (*(s + 58))) * 1e-3;
  data.velD = int((*(s + 65) << 24) | (*(s + 64) << 16) | (*(s + 63) << 8) | (*(s + 62))) * 1e-3;
}

void Ublox::decode(NavVelnedPayload& data) const
{
  if (parser_->getLatestMsg() != Ublox::NAV_VELNED)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.velN = int((*(s + 13) << 24) | (*(s + 12) << 16) | (*(s + 11) << 8) | (*(s + 10))) * 1e-2;
  data.velE = int((*(s + 17) << 24) | (*(s + 16) << 16) | (*(s + 15) << 8) | (*(s + 14))) * 1e-2;
  data.velD = int((*(s + 21) << 24) | (*(s + 20) << 16) | (*(s + 19) << 8) | (*(s + 18))) * 1e-2;
}

void Ublox::decode(NavTimegpsPayload&) const
{
  throw;  // TODO
}

void Ublox::decode(NavTimeutcPayload& data) const
{
  if (parser_->getLatestMsg() != Ublox::NAV_TIMEUTC)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.tAcc = uint32_t((*(s + 13) << 24) | (*(s + 12) << 16) | (*(s + 11) << 8) | (*(s + 10)));
  data.nano = int((*(s + 17) << 24) | (*(s + 16) << 16) | (*(s + 15) << 8) | (*(s + 14)));
  data.year = uint16_t((*(s + 19) << 8) | (*(s + 18)));
  data.month = uint8_t(*(s + 20));
  data.day = uint8_t(*(s + 21));
  data.hour = uint8_t(*(s + 22));
  data.min = uint8_t(*(s + 23));
  data.sec = uint8_t(*(s + 24));

  const auto valid = uint8_t(*(s + 25));
  data.validUTC = (valid >> 2) & 1;
}

void Ublox::decode(NavCovPayload& data) const
{
  if (parser_->getLatestMsg() != Ublox::NAV_COV)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.posCovNN =
    decodeBinary32((*(s + 25) << 24) | (*(s + 24) << 16) | (*(s + 23) << 8) | (*(s + 22)));
  data.posCovNE =
    decodeBinary32((*(s + 29) << 24) | (*(s + 28) << 16) | (*(s + 27) << 8) | (*(s + 26)));
  data.posCovND =
    decodeBinary32((*(s + 33) << 24) | (*(s + 32) << 16) | (*(s + 31) << 8) | (*(s + 30)));
  data.posCovEE =
    decodeBinary32((*(s + 37) << 24) | (*(s + 36) << 16) | (*(s + 35) << 8) | (*(s + 34)));
  data.posCovED =
    decodeBinary32((*(s + 41) << 24) | (*(s + 40) << 16) | (*(s + 39) << 8) | (*(s + 38)));
  data.posCovDD =
    decodeBinary32((*(s + 45) << 24) | (*(s + 44) << 16) | (*(s + 43) << 8) | (*(s + 42)));
  data.velCovNN =
    decodeBinary32((*(s + 49) << 24) | (*(s + 48) << 16) | (*(s + 47) << 8) | (*(s + 46)));
  data.velCovNE =
    decodeBinary32((*(s + 53) << 24) | (*(s + 52) << 16) | (*(s + 51) << 8) | (*(s + 50)));
  data.velCovND =
    decodeBinary32((*(s + 57) << 24) | (*(s + 56) << 16) | (*(s + 55) << 8) | (*(s + 54)));
  data.velCovEE =
    decodeBinary32((*(s + 61) << 24) | (*(s + 60) << 16) | (*(s + 59) << 8) | (*(s + 58)));
  data.velCovED =
    decodeBinary32((*(s + 65) << 24) | (*(s + 64) << 16) | (*(s + 63) << 8) | (*(s + 62)));
  data.velCovDD =
    decodeBinary32((*(s + 69) << 24) | (*(s + 68) << 16) | (*(s + 67) << 8) | (*(s + 66)));
}

void Ublox::decode(AckNakPayload& data) const
{
  if (parser_->getLatestMsg() != Ublox::ACK_NAK)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.clsID = uint8_t(*(s + 6));
  data.msgID = uint8_t(*(s + 7));
}

void Ublox::decode(AckAckPayload& data) const
{
  if (parser_->getLatestMsg() != Ublox::ACK_ACK)
  {
    throw runtime_error("Message type mismatch.");
  }

  const auto msg = parser_->getMessage();
  const auto pos = parser_->getPosition() - parser_->getLength();
  const auto s = msg + pos;

  data.clsID = uint8_t(*(s + 6));
  data.msgID = uint8_t(*(s + 7));
}

void Ublox::decode(MonHwPayload&) const
{
  throw;  // TODO
}

void Ublox::decode(MonHw2Payload&) const
{
  throw;  // TODO
}

bool Ublox::sendMessage(uint8_t msg_class, uint8_t msg_id, void* msg, uint16_t size)
{
  uint8_t buffer[kUbxBufferLength];

  UbxHeader header;
  header.preamble1 = PREAMBLE1;
  header.preamble2 = PREAMBLE2;
  header.msg_class = msg_class;
  header.msg_id = msg_id;
  header.length = size;

  auto offset = spliceMemory(buffer, &header, sizeof(UbxHeader));
  offset = spliceMemory(buffer, msg, size, offset);

  auto checksum = calculateCheckSum(buffer, offset);
  offset = spliceMemory(buffer, &checksum, sizeof(CheckSum), offset);

  return spi_dev_.transfer(buffer, nullptr, offset);
}

int Ublox::spliceMemory(uint8_t* dest, const void* const src, size_t size, int dest_offset)
{
  memmove(dest + dest_offset, src, size);
  return dest_offset + size;
}

Ublox::CheckSum Ublox::calculateCheckSum(uint8_t* message, size_t size) const
{
  CheckSum checksum;
  checksum.CK_A = checksum.CK_B = 0;

  for (size_t i = kPreambleOffset; i < size; ++i)
  {
    checksum.CK_A += message[i];
    checksum.CK_B += checksum.CK_A;
  }
  return checksum;
}

bool Ublox::configureGnss(uint8_t gnss_id, uint8_t res_track_ch, uint8_t max_track_ch, bool enable)
{
  assert(max_track_ch >= res_track_ch);

  CfgGnss cfg_gnss;
  memset(&cfg_gnss, 0, sizeof(CfgGnss));

  cfg_gnss.msgVer = 0x00;
  cfg_gnss.numTrkChUse = 0xFF;  // 使えるチャンネルは全て使う
  cfg_gnss.numConfigBlocks = 1;

  cfg_gnss.block.gnssId = gnss_id;
  cfg_gnss.block.resTrkCh = res_track_ch;
  cfg_gnss.block.maxTrkCh = max_track_ch;
  cfg_gnss.block.flags = enable ? (0x01 << 16) | 0x01 : 0;  // M8シリーズはL1A/Cのみ受信可 (1.5節)

  if (!sendMessage(CLASS_CFG, ID_CFG_GNSS, &cfg_gnss, sizeof(CfgGnss)))
  {
    throw runtime_error("Failed to send message.");
  }

  return waitForAcknowledge(CLASS_CFG, ID_CFG_GNSS);
}

bool Ublox::waitForAcknowledge(uint8_t cls, uint8_t id)
{
  AckAckPayload ack;
  AckNakPayload nak;

  const auto start_time = system_clock::now();
  while (duration_cast<microseconds>(system_clock::now() - start_time).count() < kWaitForGnssAck)
  {
    const auto msg = update();
    switch (msg)
    {
      case Ublox::ACK_ACK:
        decode(ack);
        if (ack.clsID == cls && ack.msgID == id)
          return true;
        break;
      case Ublox::ACK_NAK:
        decode(nak);
        if (nak.clsID == cls && nak.msgID == id)
          return false;
        break;
      default:
        break;
    }
  }

  throw runtime_error("Failed to get acknowledgement message.");
}
