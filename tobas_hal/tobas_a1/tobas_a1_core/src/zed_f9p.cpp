#include <cstring>
#include <cassert>

#include <tobas_algorithm/binary.hpp>

#include "../include/tobas_a1_core/zed_f9p.hpp"
#include "../include/tobas_a1_core/constants.hpp"

#define NOT_IMPLEMENTED "Not implemented."
#define NOT_RECEIVABLE "Not receivable."

using namespace std;
using namespace chrono;

namespace a1
{
ZEDF9P::ZEDF9P() : rate_(microseconds(kReqInterval))
{
}

bool ZEDF9P::initialize()
{
  // Initialize SPI device
  if (!spi_dev_.initialize(spi_device::kGnssDev, kSpiClockFreq, kUbxBufferLength))
    return false;

  // Disable unnecessary interfaces
  // D_SELピンをオフにしているため，I2CとUARTは始めからオフになっているはず．
  if (!enableUSB(false))
    return false;

  // Disable unnecessary protocols
  if (!enableProtocol(NMEA, false))
    return false;
  if (!enableProtocol(RTCM3X, false))
    return false;
  if (!enableProtocol(SPARTN, false))
    return false;

  return true;
}

bool ZEDF9P::update()
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
    if (!spi_dev_.transfer(1))
      return false;
    // stopwatch_.stop();

    // Scanner checks the message structure with every byte received
    // ほとんど無意味な情報だが，スタックされていくためスリープせず全て読み出す必要がある
    status = scanner_.update(spi_dev_.rx[0]);

    // SPIリクエストの間隔が短すぎると正しくデータが取得できないため，一定の間隔以上になるようスリープ．
    rate_.sleep();
  }

  if (!verifyMessage())
    return false;

  return true;
}

bool ZEDF9P::enableMsg(ubx_class_t cls, uint8_t id, bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  switch (cls)
  {
    case CLASS_MON:
    {
      cerr << NOT_IMPLEMENTED << endl;  // TODO
      return false;
    }
    case CLASS_NAV:
    {
      switch (id)
      {
        case CLOCK:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x69);  // CFG-MSGOUT-UBX_NAV_CLOCK_SPI
          break;
        case COV:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x87);  // CFG-MSGOUT-UBX_NAV_COV_SPI
          break;
        case DOP:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x3C);  // CFG-MSGOUT-UBX_NAV_DOP_SPI
          break;
        case EOE:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x63);  // CFG-MSGOUT-UBX_NAV_EOE_SPI
          break;
        case GEOFENCE:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0xA5);  // CFG-MSGOUT-UBX_NAV_GEOFENCE_SPI
          break;
        case HPPOSECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x32);  // CFG-MSGOUT-UBX_NAV_HPPOSECEF_SPI
          break;
        case HPPOSLLH:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x37);  // CFG-MSGOUT-UBX_NAV_HPPOSLLH_SPI
          break;
        case ODO:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x82);  // CFG-MSGOUT-UBX_NAV_ODO_SPI
          break;
        case ORB:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x14);  // CFG-MSGOUT-UBX_NAV_ORB_SPI
          break;
        case PL:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x19);  // CFG-MSGOUT-UBX_NAV_PL_SPI
          break;
        case POSECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x28);  // CFG-MSGOUT-UBX_NAV_POSECEF_SPI
          break;
        case POSLLH:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x2D);  // CFG-MSGOUT-UBX_NAV_POSLLH_SPI
          break;
        case PVT:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x0A);  // CFG-MSGOUT-UBX_NAV_PVT_SPI
          break;
        case RELPOSNED:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x91);  // CFG-MSGOUT-UBX_NAV_RELPOSNED_SPI
          break;
        case RESETODO:
          cerr << NOT_RECEIVABLE << endl;
          return;
        case SAT:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x19);  // CFG-MSGOUT-UBX_NAV_SAT_SPI
          break;
        case SBAS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x6E);  // CFG-MSGOUT-UBX_NAV_SBAS_SPI
          break;
        case SIG:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x49);  // CFG-MSGOUT-UBX_NAV_SIG_SPI
          break;
        case SLAS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x3A);  // CFG-MSGOUT-UBX_NAV_SLAS_SPI
          break;
        case STATUS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x1E);  // CFG-MSGOUT-UBX_NAV_STATUS_SPI
          break;
        case SVIN:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x8C);  // CFG-MSGOUT-UBX_NAV_SVIN_SPI
          break;
        case TIMEBDS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x55);  // CFG-MSGOUT-UBX_NAV_TIMEBDS_SPI
          break;
        case TIMEGAL:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x5A);  // CFG-MSGOUT-UBX_NAV_TIMEGAL_SPI
          break;
        case TIMEGLO:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x50);  // CFG-MSGOUT-UBX_NAV_TIMEGLO_SPI
          break;
        case TIMEGPS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x4B);  // CFG-MSGOUT-UBX_NAV_TIMEGPS_SPI
          break;
        case TIMELS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x64);  // CFG-MSGOUT-UBX_NAV_TIMELS_SPI
          break;
        case TIMEQZSS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x8A);  // CFG-MSGOUT-UBX_NAV_TIMEQZSS_SPI
          break;
        case TIMEUTC:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x5F);  // CFG-MSGOUT-UBX_NAV_TIMEUTC_SPI
          break;
        case VELECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x41);  // CFG-MSGOUT-UBX_NAV_VELECEF_SPI
          break;
        case VELNED:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x46);  // CFG-MSGOUT-UBX_NAV_VELNED_SPI
          break;
        default:
          cerr << NOT_IMPLEMENTED << endl;  // TODO
          return false;
      }
      break;
    }
    default:
    {
      cerr << NOT_IMPLEMENTED << endl;  // TODO
      return false;
    }
  }

  cfg.data[0].value = enable ? 1 : 0;  // Enableならば最大レート

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::configureDynamicsModel(dynamics_model_t model)
{
  CfgValSet<uint8_t, 1> cfg;

  // CFG-NAVSPG-DYNMODEL
  cfg.data[0].key = configKeyID(ONE_BYTE, CFG_NAVSPG, 0x21);
  cfg.data[0].value = model;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::configureMeasurementRate(uint16_t period_ms)
{
  CfgValSet<uint16_t, 1> cfg;

  // CFG-RATE-MEAS
  cfg.data[0].key = configKeyID(TWO_BYTES, CFG_RATE, 0x01);
  cfg.data[0].value = period_ms;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGPS(bool enable)
{
  CfgValSet<uint8_t, 3> cfg;

  // CFG-SIGNAL-GPS-ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x1F);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-GPS_L1CA_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x01);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-GPS_L2C_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x03);
  cfg.data[2].value = enable;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableSBAS(bool enable)
{
  CfgValSet<uint8_t, 2> cfg;

  // CFG-SIGNAL-SBAS-ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x20);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-SBAS_L1CA_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x05);
  cfg.data[1].value = enable;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGalileo(bool enable)
{
  CfgValSet<uint8_t, 3> cfg;

  // CFG-SIGNAL-GAL-ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x21);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-GAL_E1_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x07);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-GAL_E5B_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x0A);
  cfg.data[2].value = enable;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableBeiDou(bool enable)
{
  CfgValSet<uint8_t, 3> cfg;

  // CFG-SIGNAL-BDS-ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x22);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-BDS_B1_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x0D);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-BDS_B2_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x0E);
  cfg.data[2].value = enable;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableQZSS(bool enable)
{
  CfgValSet<uint8_t, 4> cfg;

  // CFG-SIGNAL-QZSS-ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x24);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-QZSS_L1CA_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x12);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-QZSS_L1S_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x14);
  cfg.data[2].value = enable;

  // CFG-SIGNAL-QZSS_L2C_ENA
  cfg.data[3].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x15);
  cfg.data[3].value = enable;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGLONASS(bool enable)
{
  CfgValSet<uint8_t, 3> cfg;

  // CFG-SIGNAL-GLO-ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x25);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-GLO_L1_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x18);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-GLO_L2_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x1A);
  cfg.data[2].value = enable;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableProtocol(cfg_protocol_t prot, bool enable)
{
  CfgValSet<uint8_t, 2> cfg;

  // CFG-SPIINPROT-XXX
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SPIINPROT, prot);
  cfg.data[0].value = enable;

  // CFG-SPIOUTPROT-XXX
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SPIOUTPROT, prot);
  cfg.data[1].value = enable;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::setAntennaLength(uint8_t length_m)
{
  CfgValSet<uint16_t, 1> cfg;

  // CFG-TP-ANT_CABLEDELAY
  cfg.data[0].key = configKeyID(TWO_BYTES, CFG_RATE, 0x01);
  cfg.data[0].value = length_m * kRG174CableDelay;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableUSB(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  // CFG-USB-ENABLED
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_USB, 0x01);
  cfg.data[0].value = enable;

  return configure(VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::decode(payload::ACK_NAK& data) const
{
  if (!latestMessageTypeMatch(CLASS_ACK, NAK))
    return false;

  const auto p = scanner_.getPayload();

  data.clsID = algo::decodeU8(p + 0);
  data.msgID = algo::decodeU8(p + 1);

  return true;
}

bool ZEDF9P::decode(payload::ACK_ACK& data) const
{
  if (!latestMessageTypeMatch(CLASS_ACK, ACK))
    return false;

  const auto p = scanner_.getPayload();

  data.clsID = algo::decodeU8(p + 0);
  data.msgID = algo::decodeU8(p + 1);

  return true;
}

bool ZEDF9P::decode(payload::NAV_POSLLH& data) const
{
  if (!latestMessageTypeMatch(CLASS_NAV, POSLLH))
    return false;

  const auto p = scanner_.getPayload();

  data.iTOW = algo::decodeU32(p + 0);

  data.lon = algo::decodeI32(p + 4) * 1e-7;
  data.lat = algo::decodeI32(p + 8) * 1e-7;
  data.height = algo::decodeI32(p + 12);
  data.hMSL = algo::decodeI32(p + 16);

  data.hAcc = algo::decodeU32(p + 20);
  data.vAcc = algo::decodeU32(p + 24);

  return true;
}

bool ZEDF9P::decode(payload::NAV_PVT& data) const
{
  if (!latestMessageTypeMatch(CLASS_NAV, PVT))
    return false;

  const auto p = scanner_.getPayload();

  data.iTOW = algo::decodeU32(p + 0);

  data.year = algo::decodeU16(p + 4);
  data.month = algo::decodeU8(p + 6);
  data.day = algo::decodeU8(p + 7);
  data.hour = algo::decodeU8(p + 8);
  data.min = algo::decodeU8(p + 9);
  data.sec = algo::decodeU8(p + 10);

  const auto valid = algo::decodeU8(p + 11);
  data.validDate = (valid >> 0) & 1;
  data.validTime = (valid >> 1) & 1;
  data.fullyResolved = (valid >> 2) & 1;
  data.validMag = (valid >> 3) & 1;

  data.tAcc = algo::decodeU32(p + 12);
  data.nano = algo::decodeI32(p + 16);

  data.fixType = static_cast<payload::NAV_PVT::fix_type_t>(algo::decodeU8(p + 20));

  const auto flags = algo::decodeU8(p + 21);
  data.gnssFixOk = (flags >> 0) & 1;
  data.diffSoln = (flags >> 1) & 1;
  data.psmState = static_cast<payload::NAV_PVT::psm_state_t>((flags >> 2) & 0b111);
  data.headVehValid = (flags >> 5) & 1;
  data.carrSoln = static_cast<payload::NAV_PVT::carr_soln_t>((flags >> 6) & 0b11);

  const auto flags2 = algo::decodeU8(p + 22);
  data.confirmedAvai = (flags2 >> 5) & 1;
  data.confirmedDate = (flags2 >> 6) & 1;
  data.confirmedTime = (flags2 >> 7) & 1;

  data.numSV = algo::decodeU8(p + 23);

  data.lon = algo::decodeI32(p + 24) * 1e-7;
  data.lat = algo::decodeI32(p + 28) * 1e-7;
  data.height = algo::decodeI32(p + 32);
  data.hMSL = algo::decodeI32(p + 36);
  data.hAcc = algo::decodeU32(p + 40);
  data.vAcc = algo::decodeU32(p + 44);
  data.velN = algo::decodeI32(p + 48);
  data.velE = algo::decodeI32(p + 52);
  data.velD = algo::decodeI32(p + 56);
  data.gSpeed = algo::decodeI32(p + 60);
  data.headMot = algo::decodeI32(p + 64) * 1e-5;
  data.sAcc = algo::decodeU32(p + 68);
  data.headAcc = algo::decodeU32(p + 72) * 1e-5;
  data.pDOP = algo::decodeU16(p + 76) * 1e-2;

  const auto flags3 = algo::decodeU8(p + 78);
  data.invalidLlh = (flags3 >> 0) & 1;

  data.headVeh = algo::decodeI32(p + 84) * 1e-5;
  data.magDec = algo::decodeI16(p + 88) * 1e-2;
  data.magAcc = algo::decodeU16(p + 90) * 1e-2;

  return true;
}

bool ZEDF9P::decode(payload::NAV_STATUS& data) const
{
  if (!latestMessageTypeMatch(CLASS_NAV, STATUS))
    return false;

  const auto p = scanner_.getPayload();

  data.iTOW = algo::decodeU32(p + 0);
  data.gpsFix = algo::decodeU8(p + 4);

  const auto flags = algo::decodeU8(p + 5);
  data.gpsFixOk = (flags >> 0) & 1;
  data.diffSoln = (flags >> 1) & 1;
  data.wknSet = (flags >> 2) & 1;
  data.towSet = (flags >> 3) & 1;

  const auto fixStat = algo::decodeU8(p + 6);
  data.diffCorr = (fixStat >> 0) & 1;
  data.mapMatching = static_cast<payload::NAV_STATUS::map_matching_t>((fixStat >> 6) & 0b11);

  const auto flags2 = algo::decodeU8(p + 7);
  data.psmState = static_cast<payload::NAV_STATUS::psm_state_t>((flags2 >> 0) & 0b11);
  data.spoofDetState = static_cast<payload::NAV_STATUS::spoof_det_state>((flags2 >> 3) & 0b11);

  data.ttff = algo::decodeU32(p + 8);
  data.msss = algo::decodeU32(p + 12);

  return true;
}

bool ZEDF9P::decode(payload::NAV_VELNED& data) const
{
  if (!latestMessageTypeMatch(CLASS_NAV, VELNED))
    return false;

  const auto p = scanner_.getPayload();

  data.iTOW = algo::decodeU32(p + 0);

  data.velN = algo::decodeI32(p + 4);
  data.velE = algo::decodeI32(p + 8);
  data.velD = algo::decodeI32(p + 12);

  data.speed = algo::decodeU32(p + 16);
  data.gSpeed = algo::decodeU32(p + 20);
  data.heading = algo::decodeI32(p + 24) * 1e-5;

  data.sAcc = algo::decodeU32(p + 28);
  data.cAcc = algo::decodeU32(p + 32) * 1e-5;

  return true;
}

bool ZEDF9P::sendMessage(ubx_class_t cls, uint8_t id, const void* msg, uint16_t size)
{
  UbxHeader header;
  header.sync1 = kUbxSync1;
  header.sync2 = kUbxSync2;
  header.cls = cls;
  header.id = id;
  header.length = size;

  const auto payload_pos = spliceMemory(spi_dev_.tx, &header, sizeof(UbxHeader), 0);
  const auto checksum_pos = spliceMemory(spi_dev_.tx, msg, size, payload_pos);

  const auto ck = computeChecksum(spi_dev_.tx, checksum_pos);
  const auto message_length = spliceMemory(spi_dev_.tx, &ck, sizeof(CheckSum), checksum_pos);

  if (!spi_dev_.transfer(message_length))
    return false;

  return true;
}

bool ZEDF9P::waitForAcknowledge(ubx_class_t cls, uint8_t id)
{
  payload::ACK_ACK ack;
  payload::ACK_NAK nak;

  const auto cls_str = to_string(int(cls));
  const auto id_str = to_string(int(id));

  const auto start_time = system_clock::now();

  while (duration<double>(system_clock::now() - start_time).count() < kWaitForGnssAck)
  {
    if (!update())
      return false;

    if (latestClass() != CLASS_ACK)
      continue;

    switch (latestId())
    {
      case ACK:
        if (!decode(ack))
          return false;

        if (ack.clsID == cls && ack.msgID == id)
        {
          return true;
        }
        else
        {
          cerr << "An acknowledment message for an unspecified message is received." << endl;
          return false;
        }

        break;

      case NAK:
        if (!decode(nak))
          return false;

        if (nak.clsID == cls && nak.msgID == id)
        {
          cerr << "Configuration is rejected: (class, id) = (" << cls_str << ", " << id_str << ")" << endl;
          return false;
        }
        else
        {
          cerr << "A non-acknowledment message for an unspecified message is received." << endl;
          return false;
        }

        break;

      default:
        cerr << "Unexpected ACK ID: " << (int)latestId() << endl;
        break;
    }
  }

  cerr << "Acknowledment message not received: (class, id) = (" << cls_str << ", " << id_str << ")" << endl;
  return false;
}

bool ZEDF9P::configure(ubx_cfg_id_t cfg_id, const void* msg, uint16_t size)
{
  if (!sendMessage(CLASS_CFG, cfg_id, msg, size))
    return false;

  return waitForAcknowledge(CLASS_CFG, cfg_id);
}

bool ZEDF9P::latestMessageTypeMatch(ubx_class_t cls, uint8_t id) const
{
  if (latestClass() != cls || latestId() != id)
  {
    cerr << "Message type mismatch." << endl;
    return false;
  }

  return true;
}

bool ZEDF9P::verifyMessage() const
{
  // Sync chars
  if (*scanner_.getSync1() != kUbxSync1 || *scanner_.getSync2() != kUbxSync2)
  {
    cerr << "The current message is not UBX format." << endl;
    return false;
  }

  // Checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (auto x = scanner_.getClass(); x < scanner_.getChecksumA(); ++x)
  {
    CK_A += *x;
    CK_B += CK_A;
  }
  if (CK_A != *scanner_.getChecksumA() || CK_B != *scanner_.getChecksumB())
  {
    cerr << "Checksum failed." << endl;
    return false;
  }

  return true;
}

ZEDF9P::CheckSum ZEDF9P::computeChecksum(const uint8_t* message, size_t checksum_pos)
{
  CheckSum ck;
  ck.CK_A = ck.CK_B = 0;

  for (size_t i = kUbxSyncLength; i < checksum_pos; ++i)
  {
    ck.CK_A += message[i];
    ck.CK_B += ck.CK_A;
  }

  return ck;
}

size_t ZEDF9P::spliceMemory(uint8_t* dest, const void* src, size_t size, size_t dest_offset)
{
  memmove(dest + dest_offset, src, size);
  return dest_offset + size;
}

uint32_t ZEDF9P::configKeyID(cfg_size_t size, cfg_group_t group, uint8_t id)
{
  return (size << 28) | (group << 16) | id;
}
}  // namespace a1
