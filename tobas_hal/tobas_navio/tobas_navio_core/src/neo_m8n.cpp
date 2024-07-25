#include <stdexcept>
#include <cstring>
#include <cassert>
#include <chrono>

#include <tobas_algorithm/binary.hpp>

#include "../include/tobas_navio_core/neo_m8n.hpp"

using namespace std;
using namespace chrono;

namespace navio
{
NEOM8N::NEOM8N() : rate_(chrono::microseconds(kSpiInterval))
{
}

bool NEOM8N::initialize()
{
  if (!spi_dev_.initialize(kDevice, kSpiClockFreq, kUbxBufferLength))
    return false;

  return true;
}

void NEOM8N::clearConfigurations()
{
  CfgCfg cfg_cfg;

  cfg_cfg.clearMask = UINT32_MAX;
  cfg_cfg.saveMask = 0;
  cfg_cfg.loadMask = 0;

  configure(ID_CFG_CFG, &cfg_cfg, sizeof(CfgCfg));
}

void NEOM8N::saveConfigurations()
{
  CfgCfg cfg_cfg;

  cfg_cfg.clearMask = 0;
  cfg_cfg.saveMask = UINT32_MAX;
  cfg_cfg.loadMask = 0;

  configure(ID_CFG_CFG, &cfg_cfg, sizeof(CfgCfg));
}

void NEOM8N::loadConfigurations()
{
  CfgCfg cfg_cfg;

  cfg_cfg.clearMask = 0;
  cfg_cfg.saveMask = 0;
  cfg_cfg.loadMask = UINT32_MAX;

  configure(ID_CFG_CFG, &cfg_cfg, sizeof(CfgCfg));
}

void NEOM8N::enableMsg(message_t msg, bool enable)
{
  CfgMsg cfg_msg;

  cfg_msg.msgClass = msg >> 8;                       // 上位8ビット
  cfg_msg.msgID = static_cast<uint8_t>(msg & 0xFF);  // 下位8ビット
  cfg_msg.rate = enable;

  configure(ID_CFG_MSG, &cfg_msg, sizeof(CfgMsg));
}

void NEOM8N::enableAllMsgs(bool enable)
{
  enableMsg(NAV_POSLLH, enable);
  enableMsg(NAV_STATUS, enable);
  enableMsg(NAV_DOP, enable);
  enableMsg(NAV_PVT, enable);
  enableMsg(NAV_VELNED, enable);
  enableMsg(NAV_TIMEGPS, enable);
  enableMsg(NAV_TIMEUTC, enable);
  enableMsg(NAV_COV, enable);

  enableMsg(MON_HW, enable);
  enableMsg(MON_HW2, enable);
}

void NEOM8N::configureSolutionRate(uint16_t meas_rate, uint16_t nav_rate, uint16_t time_ref)
{
  CfgRate cfg_rate;

  cfg_rate.measRate = meas_rate;
  cfg_rate.navRate = nav_rate;
  cfg_rate.timeRef = time_ref;

  configure(ID_CFG_RATE, &cfg_rate, sizeof(CfgRate));
}

void NEOM8N::configureDynamicsModel(dynamics_model dyn_model)
{
  CfgNav5 cfg_nav5;

  cfg_nav5.mask = 0b10;  // dynModelのみ更新
  cfg_nav5.dynModel = dyn_model;

  configure(ID_CFG_NAV5, &cfg_nav5, sizeof(CfgNav5));
}

void NEOM8N::configurePowerMode(power_setup_value mode, uint16_t period, uint16_t on_time)
{
  CfgPms cfg_pms;

  cfg_pms.powerSetupValue = mode;
  cfg_pms.period = period;
  cfg_pms.onTime = on_time;

  configure(ID_CFG_PMS, &cfg_pms, sizeof(CfgPms));
}

void NEOM8N::configureGnss_GPS(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  assert(max_track_ch >= kMinMaxTrkChForMajorGnss);
  configureGnss(GPS, res_track_ch, max_track_ch, enable);
}

void NEOM8N::configureGnss_SBAS(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  configureGnss(SBAS, res_track_ch, max_track_ch, enable);
}

void NEOM8N::configureGnss_Galileo(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  assert(max_track_ch >= kMinMaxTrkChForMajorGnss);
  configureGnss(GALILEO, res_track_ch, max_track_ch, enable);
}

void NEOM8N::configureGnss_BeiDou(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  assert(max_track_ch >= kMinMaxTrkChForMajorGnss);
  configureGnss(BEIDOU, res_track_ch, max_track_ch, enable);
}

void NEOM8N::configureGnss_QZSS(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  configureGnss(QZSS, res_track_ch, max_track_ch, enable);
}

void NEOM8N::configureGnss_GLONASS(bool enable, uint8_t res_track_ch, uint8_t max_track_ch)
{
  assert(max_track_ch >= kMinMaxTrkChForMajorGnss);
  configureGnss(GLONASS, res_track_ch, max_track_ch, enable);
}

uint16_t NEOM8N::update()
{
  int status = -1;
  spi_dev_.tx[0] = 0;
  scanner_.reset();

  // メッセージを1つスキャン
  rate_.start();
  while (status != UBXScanner::Done)
  {
    // From now on, we will send zeroes to the receiver, which it will ignore
    // However, we are simultaneously getting useful information from it
    // stopwatch_.start();
    spi_dev_.transfer(1);
    // stopwatch_.stop();

    // Scanner checks the message structure with every byte received
    // ほとんど無意味な情報だが，スタックされていくためスリープせず全て読み出す必要がある
    status = scanner_.update(spi_dev_.rx[0]);

    // SPIリクエストの間隔が短すぎると正しくデータが取得できないため，一定の間隔以上になるようスリープ．
    rate_.sleep();
  }

  verifyMessage();

  // 取得したメッセージのID (Class + ID) を返す
  return latest_msg_ = (*scanner_.getClass()) << 8 | (*scanner_.getId());
}

void NEOM8N::decode(NavPosllhPayload& data) const
{
  if (latest_msg_ != NEOM8N::NAV_POSLLH)
    throw runtime_error("Message type mismatch.");

  const auto p = scanner_.getPayload();

  data.lon = int((*(p + 7) << 24) | (*(p + 6) << 16) | (*(p + 5) << 8) | (*(p + 4))) * 1e-7;
  data.lat = int((*(p + 11) << 24) | (*(p + 10) << 10) | (*(p + 9) << 8) | (*(p + 8))) * 1e-7;
  data.hMSL = int((*(p + 19) << 24) | (*(p + 18) << 16) | (*(p + 17) << 8) | (*(p + 16))) * 1e-3;
}

void NEOM8N::decode(NavStatusPayload& data) const
{
  if (latest_msg_ != NEOM8N::NAV_STATUS)
    throw runtime_error("Message type mismatch.");

  const auto p = scanner_.getPayload();

  data.gpsFix = uint8_t(*(p + 4));

  const auto flags = uint8_t(*(p + 5));
  data.gpsFixOk = (flags >> 0) & 1;
}

void NEOM8N::decode(NavDopPayload&) const
{
  throw;  // TODO
}

void NEOM8N::decode(NavPvtPayload& data) const
{
  if (latest_msg_ != NEOM8N::NAV_PVT)
    throw runtime_error("Message type mismatch.");

  const auto p = scanner_.getPayload();

  data.year = uint16_t((*(p + 5) << 8) | (*(p + 4)));
  data.month = uint8_t(*(p + 6));
  data.day = uint8_t(*(p + 7));
  data.hour = uint8_t(*(p + 8));
  data.min = uint8_t(*(p + 9));
  data.sec = uint8_t(*(p + 10));

  const auto valid = uint8_t(*(p + 11));
  data.validDate = (valid >> 0) & 1;
  data.validTime = (valid >> 1) & 1;
  data.fullyResolved = (valid >> 2) & 1;
  data.validMag = (valid >> 3) & 1;

  data.tAcc = uint32_t((*(p + 15) << 24) | (*(p + 14) << 16) | (*(p + 13) << 8) | (*(p + 12)));
  data.nano = int((*(p + 19) << 24) | (*(p + 18) << 16) | (*(p + 17) << 8) | (*(p + 16)));

  data.fixType = uint8_t(*(p + 20));

  const auto flags = uint8_t(*(p + 21));
  data.gnssFixOk = (flags >> 0) & 1;

  data.lon = int((*(p + 27) << 24) | (*(p + 26) << 16) | (*(p + 25) << 8) | (*(p + 24))) * 1e-7;
  data.lat = int((*(p + 31) << 24) | (*(p + 30) << 16) | (*(p + 29) << 8) | (*(p + 28))) * 1e-7;
  data.hMSL = int((*(p + 39) << 24) | (*(p + 38) << 16) | (*(p + 37) << 8) | (*(p + 36))) * 1e-3;
  data.velN = int((*(p + 51) << 24) | (*(p + 50) << 16) | (*(p + 49) << 8) | (*(p + 48))) * 1e-3;
  data.velE = int((*(p + 55) << 24) | (*(p + 54) << 16) | (*(p + 53) << 8) | (*(p + 52))) * 1e-3;
  data.velD = int((*(p + 59) << 24) | (*(p + 58) << 16) | (*(p + 57) << 8) | (*(p + 56))) * 1e-3;
}

void NEOM8N::decode(NavVelnedPayload& data) const
{
  if (latest_msg_ != NEOM8N::NAV_VELNED)
    throw runtime_error("Message type mismatch.");

  const auto p = scanner_.getPayload();

  data.velN = int((*(p + 7) << 24) | (*(p + 6) << 16) | (*(p + 5) << 8) | (*(p + 4))) * 1e-2;
  data.velE = int((*(p + 11) << 24) | (*(p + 10) << 16) | (*(p + 9) << 8) | (*(p + 8))) * 1e-2;
  data.velD = int((*(p + 15) << 24) | (*(p + 14) << 16) | (*(p + 13) << 8) | (*(p + 12))) * 1e-2;
}

void NEOM8N::decode(NavTimegpsPayload&) const
{
  throw;  // TODO
}

void NEOM8N::decode(NavTimeutcPayload& data) const
{
  if (latest_msg_ != NEOM8N::NAV_TIMEUTC)
    throw runtime_error("Message type mismatch.");

  const auto p = scanner_.getPayload();

  data.tAcc = uint32_t((*(p + 7) << 24) | (*(p + 6) << 16) | (*(p + 5) << 8) | (*(p + 4)));
  data.nano = int((*(p + 11) << 24) | (*(p + 10) << 16) | (*(p + 9) << 8) | (*(p + 8)));
  data.year = uint16_t((*(p + 13) << 8) | (*(p + 12)));
  data.month = uint8_t(*(p + 14));
  data.day = uint8_t(*(p + 15));
  data.hour = uint8_t(*(p + 16));
  data.min = uint8_t(*(p + 17));
  data.sec = uint8_t(*(p + 18));

  const auto valid = uint8_t(*(p + 19));
  data.validUTC = (valid >> 2) & 1;
}

void NEOM8N::decode(NavCovPayload& data) const
{
  if (latest_msg_ != NEOM8N::NAV_COV)
    throw runtime_error("Message type mismatch.");

  const auto p = scanner_.getPayload();

  data.posCovNN = algo::decodeR32((*(p + 19) << 24) | (*(p + 18) << 16) | (*(p + 17) << 8) | (*(p + 16)));
  data.posCovNE = algo::decodeR32((*(p + 23) << 24) | (*(p + 22) << 16) | (*(p + 21) << 8) | (*(p + 20)));
  data.posCovND = algo::decodeR32((*(p + 27) << 24) | (*(p + 26) << 16) | (*(p + 25) << 8) | (*(p + 24)));
  data.posCovEE = algo::decodeR32((*(p + 31) << 24) | (*(p + 30) << 16) | (*(p + 29) << 8) | (*(p + 28)));
  data.posCovED = algo::decodeR32((*(p + 35) << 24) | (*(p + 34) << 16) | (*(p + 33) << 8) | (*(p + 32)));
  data.posCovDD = algo::decodeR32((*(p + 39) << 24) | (*(p + 38) << 16) | (*(p + 37) << 8) | (*(p + 36)));
  data.velCovNN = algo::decodeR32((*(p + 43) << 24) | (*(p + 42) << 16) | (*(p + 41) << 8) | (*(p + 40)));
  data.velCovNE = algo::decodeR32((*(p + 47) << 24) | (*(p + 46) << 16) | (*(p + 45) << 8) | (*(p + 44)));
  data.velCovND = algo::decodeR32((*(p + 51) << 24) | (*(p + 50) << 16) | (*(p + 49) << 8) | (*(p + 48)));
  data.velCovEE = algo::decodeR32((*(p + 55) << 24) | (*(p + 54) << 16) | (*(p + 53) << 8) | (*(p + 52)));
  data.velCovED = algo::decodeR32((*(p + 59) << 24) | (*(p + 58) << 16) | (*(p + 57) << 8) | (*(p + 56)));
  data.velCovDD = algo::decodeR32((*(p + 63) << 24) | (*(p + 62) << 16) | (*(p + 61) << 8) | (*(p + 60)));
}

void NEOM8N::decode(AckNakPayload& data) const
{
  if (latest_msg_ != NEOM8N::ACK_NAK)
    throw runtime_error("Message type mismatch.");

  const auto p = scanner_.getPayload();

  data.clsID = uint8_t(*(p + 0));
  data.msgID = uint8_t(*(p + 1));
}

void NEOM8N::decode(AckAckPayload& data) const
{
  if (latest_msg_ != NEOM8N::ACK_ACK)
    throw runtime_error("Message type mismatch.");

  const auto p = scanner_.getPayload();

  data.clsID = uint8_t(*(p + 0));
  data.msgID = uint8_t(*(p + 1));
}

void NEOM8N::decode(MonHwPayload&) const
{
  throw;  // TODO
}

void NEOM8N::decode(MonHw2Payload&) const
{
  throw;  // TODO
}

void NEOM8N::sendMessage(uint8_t cls, uint8_t id, void* msg, uint16_t size)
{
  UbxHeader header;
  header.sync1 = kUbxSync1;
  header.sync2 = kUbxSync2;
  header.cls = cls;
  header.id = id;
  header.length = size;

  const auto payload_pos = spliceMemory(spi_dev_.tx, &header, sizeof(UbxHeader));
  const auto checksum_pos = spliceMemory(spi_dev_.tx, msg, size, payload_pos);

  const auto checksum = computeChecksum(spi_dev_.tx, checksum_pos);
  const auto message_length = spliceMemory(spi_dev_.tx, &checksum, sizeof(CheckSum), checksum_pos);

  if (!spi_dev_.transfer(message_length))
    throw runtime_error("Failed to send SPI message.");
}

void NEOM8N::waitForAcknowledge(uint8_t cls, uint8_t id)
{
  AckAckPayload ack;
  AckNakPayload nak;

  const auto cls_str = to_string(int(cls));
  const auto id_str = to_string(int(id));

  const auto start_time = system_clock::now();
  while (duration<double>(system_clock::now() - start_time).count() < kWaitForGnssAck)
  {
    const auto msg = update();
    switch (msg)
    {
      case NEOM8N::ACK_ACK:
        decode(ack);
        if (ack.clsID == cls && ack.msgID == id)
          return;
        else
          throw runtime_error("An acknowledment message for an unspecified message is received.");
        break;
      case NEOM8N::ACK_NAK:
        decode(nak);
        if (nak.clsID == cls && nak.msgID == id)
          throw runtime_error("Configuration is rejected: (class, id) = (" + cls_str + ", " + id_str + ")");
        else
          throw runtime_error("A non-acknowledment message for an unspecified message is received.");
        break;
      default:
        break;
    }
  }

  throw runtime_error("Acknowledment message not received: (class, id) = (" + cls_str + ", " + id_str + ")");
}

void NEOM8N::configure(uint8_t cfg_id, void* msg, uint16_t size)
{
  sendMessage(CLASS_CFG, cfg_id, msg, size);
  waitForAcknowledge(CLASS_CFG, cfg_id);
}

void NEOM8N::configureGnss(uint8_t gnss_id, uint8_t res_track_ch, uint8_t max_track_ch, bool enable)
{
  assert(max_track_ch >= res_track_ch);

  CfgGnss cfg_gnss;

  cfg_gnss.numTrkChUse = 0xFF;  // 使えるチャンネルは全て使う

  cfg_gnss.block.gnssId = gnss_id;
  cfg_gnss.block.resTrkCh = res_track_ch;
  cfg_gnss.block.maxTrkCh = max_track_ch;
  cfg_gnss.block.flags = enable ? (0x01 << 16) | 0x01 : 0;  // M8シリーズはL1A/Cのみ受信可 (1.5節)

  configure(ID_CFG_GNSS, &cfg_gnss, sizeof(CfgGnss));
}

void NEOM8N::verifyMessage()
{
  // Sync chars
  if (*scanner_.getSync1() != kUbxSync1 || *scanner_.getSync2() != kUbxSync2)
    throw runtime_error("The current message is not UBX format.");

  // Checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (auto x = scanner_.getClass(); x < scanner_.getChecksumA(); ++x)
  {
    CK_A += *x;
    CK_B += CK_A;
  }
  if (CK_A != *scanner_.getChecksumA() || CK_B != *scanner_.getChecksumB())
    throw runtime_error("Checksum failed.");
}

NEOM8N::CheckSum NEOM8N::computeChecksum(uint8_t* message, size_t checksum_pos)
{
  CheckSum checksum;
  checksum.CK_A = checksum.CK_B = 0;

  for (size_t i = kUbxSyncLength; i < checksum_pos; ++i)
  {
    checksum.CK_A += message[i];
    checksum.CK_B += checksum.CK_A;
  }

  return checksum;
}

int NEOM8N::spliceMemory(uint8_t* dest, const void* const src, size_t size, int dest_offset)
{
  memmove(dest + dest_offset, src, size);
  return dest_offset + size;
}
}  // namespace navio
