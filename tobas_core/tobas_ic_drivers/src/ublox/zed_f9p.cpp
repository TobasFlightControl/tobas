#include "tobas_ic_drivers/ublox/zed_f9p.hpp"

#include <cassert>
#include <cstring>

#define NOT_IMPLEMENTED "Not implemented."
#define NOT_RECEIVABLE "Not receivable."

using namespace std;
using namespace chrono;

namespace ublox
{
ZEDF9P::ZEDF9P() : rate_(kReqInterval)
{
}

bool ZEDF9P::initialize(const char* spi_device)
{
  // Initialize SPI device
  if (!spi_.initialize(spi_device, tx_buf_, rx_buf_, kSPIClockFreq)) {
    return false;
  }

  return true;
}

bool ZEDF9P::update(bool nonblock)
{
  scanner_.reset();

  if (nonblock) {
    // スタートバイトを確認
    if (!spi_.transfer(1)) {
      return false;
    }
    if (!scanner_.update(rx_buf_[0])) {
      return false;
    }

    // データが来てなければ終了
    if (scanner_.state() == UBXScanner::Sync1) {
      return false;
    }
  }

  // メッセージを1つスキャン
  rate_.start();
  while (scanner_.state() != UBXScanner::Done) {
    if (!spi_.transfer(1)) {
      return false;
    }
    if (!scanner_.update(rx_buf_[0])) {
      return false;
    }

    // SPIリクエストの間隔が短すぎると正しくデータが取得できないため，一定の間隔以上になるようスリープ．
    rate_.sleep();
  }

  if (!verifyMessage()) {
    return false;
  }

  return true;
}

bool ZEDF9P::enableMsg(ubx_class_t cls, uint8_t id, bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  switch (cls) {
    case CLASS_MON: {
      cerr << NOT_IMPLEMENTED << endl;  // TODO
      return false;
    }
    case CLASS_NAV: {
      switch (id) {
        case NAV_CLOCK:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x69);  // CFG-MSGOUT-UBX_NAV_CLOCK_SPI
          break;
        case NAV_COV:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x87);  // CFG-MSGOUT-UBX_NAV_COV_SPI
          break;
        case NAV_DOP:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x3C);  // CFG-MSGOUT-UBX_NAV_DOP_SPI
          break;
        case NAV_EOE:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x63);  // CFG-MSGOUT-UBX_NAV_EOE_SPI
          break;
        case NAV_GEOFENCE:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0xA5);  // CFG-MSGOUT-UBX_NAV_GEOFENCE_SPI
          break;
        case NAV_HPPOSECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x32);  // CFG-MSGOUT-UBX_NAV_HPPOSECEF_SPI
          break;
        case NAV_HPPOSLLH:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x37);  // CFG-MSGOUT-UBX_NAV_HPPOSLLH_SPI
          break;
        case NAV_ODO:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x82);  // CFG-MSGOUT-UBX_NAV_ODO_SPI
          break;
        case NAV_ORB:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x14);  // CFG-MSGOUT-UBX_NAV_ORB_SPI
          break;
        case NAV_PL:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x19);  // CFG-MSGOUT-UBX_NAV_PL_SPI
          break;
        case NAV_POSECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x28);  // CFG-MSGOUT-UBX_NAV_POSECEF_SPI
          break;
        case NAV_POSLLH:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x2D);  // CFG-MSGOUT-UBX_NAV_POSLLH_SPI
          break;
        case NAV_PVT:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x0A);  // CFG-MSGOUT-UBX_NAV_PVT_SPI
          break;
        case NAV_RELPOSNED:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x91);  // CFG-MSGOUT-UBX_NAV_RELPOSNED_SPI
          break;
        case NAV_RESETODO:
          cerr << NOT_RECEIVABLE << endl;
          return false;
        case NAV_SAT:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x19);  // CFG-MSGOUT-UBX_NAV_SAT_SPI
          break;
        case NAV_SBAS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x6E);  // CFG-MSGOUT-UBX_NAV_SBAS_SPI
          break;
        case NAV_SIG:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x49);  // CFG-MSGOUT-UBX_NAV_SIG_SPI
          break;
        case NAV_SLAS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x3A);  // CFG-MSGOUT-UBX_NAV_SLAS_SPI
          break;
        case NAV_STATUS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x1E);  // CFG-MSGOUT-UBX_NAV_STATUS_SPI
          break;
        case NAV_SVIN:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x8C);  // CFG-MSGOUT-UBX_NAV_SVIN_SPI
          break;
        case NAV_TIMEBDS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x55);  // CFG-MSGOUT-UBX_NAV_TIMEBDS_SPI
          break;
        case NAV_TIMEGAL:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x5A);  // CFG-MSGOUT-UBX_NAV_TIMEGAL_SPI
          break;
        case NAV_TIMEGLO:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x50);  // CFG-MSGOUT-UBX_NAV_TIMEGLO_SPI
          break;
        case NAV_TIMEGPS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x4B);  // CFG-MSGOUT-UBX_NAV_TIMEGPS_SPI
          break;
        case NAV_TIMELS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x64);  // CFG-MSGOUT-UBX_NAV_TIMELS_SPI
          break;
        case NAV_TIMEQZSS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x8A);  // CFG-MSGOUT-UBX_NAV_TIMEQZSS_SPI
          break;
        case NAV_TIMEUTC:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x5F);  // CFG-MSGOUT-UBX_NAV_TIMEUTC_SPI
          break;
        case NAV_VELECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x41);  // CFG-MSGOUT-UBX_NAV_VELECEF_SPI
          break;
        case NAV_VELNED:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x46);  // CFG-MSGOUT-UBX_NAV_VELNED_SPI
          break;
        default:
          cerr << NOT_IMPLEMENTED << endl;  // TODO
          return false;
      }
      break;
    }
    default: {
      cerr << NOT_IMPLEMENTED << endl;  // TODO
      return false;
    }
  }

  cfg.data[0].value = enable ? 1 : 0;  // Enableならば最大レート

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::configureDynamicsModel(dynamics_model_t model)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BYTE, CFG_NAVSPG, 0x21);  // CFG-NAVSPG-DYNMODEL
  cfg.data[0].value = model;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::configureMeasurementRate(uint16_t period_ms)
{
  CfgValSet<uint16_t, 1> cfg;

  cfg.data[0].key = configKeyID(TWO_BYTES, CFG_RATE, 0x01);  // CFG-RATE-MEAS
  cfg.data[0].value = period_ms;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGps()
{
  // Enable GPS
  if (!enableGps(true)) {
    cerr << "Failed to enable GPS." << endl;
    return false;
  }

  // Enable L1 band
  if (!enableGpsL1()) {
    cerr << "Failed to enable GPS L1." << endl;
    return false;
  }

  // Try to enable L2 band
  if (enableGpsL2()) {
    cout << "GPS L1/L2 is enabled." << endl;
    return true;
  }

  // Try to enable L5 band
  if (enableGpsL5()) {
    cout << "GPS L1/L5 is enabled." << endl;
    return true;
  }

  cerr << "Failed to enable either GPS L2 or L5 bands." << endl;
  return false;
}

bool ZEDF9P::disableGps()
{
  return enableGps(false);
}

bool ZEDF9P::enableSbas()
{
  // Enable SBAS
  if (!enableSbas(true)) {
    cerr << "Failed to enable SBAS." << endl;
    return false;
  }

  // Enable L1 band
  if (!enableSbasL1()) {
    cerr << "Failed to enable SBAS L1." << endl;
    return false;
  }

  return true;
}

bool ZEDF9P::disableSbas()
{
  return enableGps(false);
}

bool ZEDF9P::enableGalileo()
{
  // Enable Galileo
  if (!enableGalileo(true)) {
    cerr << "Failed to enable Galileo." << endl;
    return false;
  }

  // Enable L1 band
  if (!enableGalileoL1()) {
    cerr << "Failed to enable Galileo L1." << endl;
    return false;
  }

  // Try to enable L2 band
  if (enableGalileoL2()) {
    cout << "Galileo L1/L2 is enabled." << endl;
    return true;
  }

  // Try to enable L5 band
  if (enableGalileoL5()) {
    cout << "Galileo L1/L5 is enabled." << endl;
    return true;
  }

  cerr << "Failed to enable either Galileo L2 or L5 bands." << endl;
  return false;
}

bool ZEDF9P::disableGalileo()
{
  return enableGalileo(false);
}

bool ZEDF9P::enableBeiDou()
{
  // Enable BeiDou
  if (!enableBeiDou(true)) {
    cerr << "Failed to enable BeiDou." << endl;
    return false;
  }

  // Enable L1 band
  if (!enableBeiDouL1()) {
    cerr << "Failed to enable BeiDou L1." << endl;
    return false;
  }

  // Try to enable L2 band
  if (enableBeiDouL2()) {
    cout << "BeiDou L1/L2 is enabled." << endl;
    return true;
  }

  // Try to enable L5 band
  if (enableBeiDouL5()) {
    cout << "BeiDou L1/L5 is enabled." << endl;
    return true;
  }

  cerr << "Failed to enable either BeiDou L2 or L5 bands." << endl;
  return false;
}

bool ZEDF9P::disableBeiDou()
{
  return enableBeiDou(false);
}

bool ZEDF9P::enableQzss()
{
  // Enable QZSS
  if (!enableQzss(true)) {
    cerr << "Failed to enable QZSS." << endl;
    return false;
  }

  // Enable L1 band
  if (!enableQzssL1()) {
    cerr << "Failed to enable QZSS L1." << endl;
    return false;
  }

  // Try to enable L2 band
  if (enableQzssL2()) {
    cout << "QZSS L1/L2 is enabled." << endl;
    return true;
  }

  // Try to enable L5 band
  if (enableQzssL5()) {
    cout << "QZSS L1/L5 is enabled." << endl;
    return true;
  }

  cerr << "Failed to enable either QZSS L2 or L5 bands." << endl;
  return false;
}

bool ZEDF9P::disableQzss()
{
  return enableQzss(false);
}

bool ZEDF9P::enableGlonass()
{
  // Enable GLONASS
  if (!enableGlonass(true)) {
    cerr << "Failed to enable GLONASS." << endl;
    return false;
  }

  // Enable L1 band
  if (!enableGlonassL1()) {
    cerr << "Failed to enable GLONASS L1." << endl;
    return false;
  }

  // Try to enable L2 band
  if (enableGlonassL2()) {
    cout << "GLONASS L1/L2 is enabled." << endl;
    return true;
  }

  cout << "GLONASS L1 is enabled." << endl;
  return true;
}

bool ZEDF9P::disableGlonass()
{
  return enableGlonass(false);
}

bool ZEDF9P::enableNavIc()
{
  // Enable NavIC
  if (!enableNavIc(true)) {
    cerr << "Failed to enable NavIC." << endl;
    return false;
  }

  // Enable L5 band
  if (!enableNavIcL5()) {
    cerr << "Failed to enable NavIC L5." << endl;
    return false;
  }

  cout << "NavIC L5 is enabled." << endl;
  return false;
}

bool ZEDF9P::disableNavIc()
{
  return enableNavIc(false);
}

bool ZEDF9P::enableProtocol(cfg_protocol_t prot, bool enable)
{
  CfgValSet<uint8_t, 2> cfg;

  for (size_t i = 0; i < 2; ++i) {
    cfg.data[i].value = enable;
  }

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SPIINPROT, prot);   // CFG-SPIINPROT-XXX
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SPIOUTPROT, prot);  // CFG-SPIOUTPROT-XXX

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::setAntennaLength(uint8_t length_m)
{
  CfgValSet<uint16_t, 1> cfg;

  cfg.data[0].key = configKeyID(TWO_BYTES, CFG_RATE, 0x01);  // CFG-TP-ANT_CABLEDELAY
  cfg.data[0].value = length_m * kRG174CableDelay;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableUsb(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_USB, 0x01);  // CFG-USB-ENABLED
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::sendMessage(ubx_class_t cls, uint8_t id, const void* msg, uint16_t size)
{
  UbxHeader header;
  header.sync1 = kUbxSync1;
  header.sync2 = kUbxSync2;
  header.cls = cls;
  header.id = id;
  header.length = size;

  const auto payload_pos = spliceMemory(tx_buf_, &header, sizeof(UbxHeader), 0);
  const auto checksum_pos = spliceMemory(tx_buf_, msg, size, payload_pos);

  const auto ck = computeChecksum(tx_buf_, checksum_pos);
  const auto message_length = spliceMemory(tx_buf_, &ck, sizeof(CheckSum), checksum_pos);

  if (!spi_.transfer(message_length)) {
    return false;
  }

  return true;
}

bool ZEDF9P::waitForAcknowledge(ubx_class_t cls, uint8_t id)
{
  payload::ACK_ACK ack;
  payload::ACK_NAK nak;

  const auto cls_str = to_string(int(cls));
  const auto id_str = to_string(int(id));

  const auto start_time = steady_clock::now();

  while (duration<double>(steady_clock::now() - start_time).count() < kWaitForGnssAck) {
    if (!update(false)) {
      return false;
    }

    if (latestClass() != CLASS_ACK) {
      continue;
    }

    switch (latestId()) {
      case ACK_ACK:
        ack.decode(payload());

        if (ack.clsID == cls && ack.msgID == id) {
          return true;
        }
        else {
          cerr << "An acknowledment message for an unspecified message is received." << endl;
          return false;
        }

        break;

      case ACK_NAK:
        nak.decode(payload());

        if (nak.clsID == cls && nak.msgID == id) {
          cerr << "Configuration is rejected: (class, id) = (" << cls_str << ", " << id_str << ")" << endl;
          return false;
        }
        else {
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
  if (!sendMessage(CLASS_CFG, cfg_id, msg, size)) {
    return false;
  }

  return waitForAcknowledge(CLASS_CFG, cfg_id);
}

bool ZEDF9P::verifyMessage() const
{
  // Sync chars
  if (*scanner_.getSync1() != kUbxSync1 || *scanner_.getSync2() != kUbxSync2) {
    cerr << "The current message is not UBX format." << endl;
    return false;
  }

  // Checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (auto x = scanner_.getClass(); x < scanner_.getChecksumA(); ++x) {
    CK_A += *x;
    CK_B += CK_A;
  }
  if (CK_A != *scanner_.getChecksumA() || CK_B != *scanner_.getChecksumB()) {
    cerr << "Checksum failed." << endl;
    return false;
  }

  return true;
}

bool ZEDF9P::enableGps(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x1F);  // CFG-SIGNAL-GPS_ENA
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGpsL1()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x01);  // CFG-SIGNAL-GPS_L1CA_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGpsL2()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x03);  // CFG-SIGNAL-GPS_L2C_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGpsL5()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x04);  // CFG-SIGNAL-GPS_L5_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableSbas(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x20);  // CFG-SIGNAL-SBAS_ENA
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableSbasL1()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x05);  // CFG-SIGNAL-SBAS_L1CA_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGalileo(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x21);  // CFG-SIGNAL-GAL_ENA
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGalileoL1()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x07);  // CFG-SIGNAL-GAL_E1_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGalileoL2()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x0A);  // CFG-SIGNAL-GAL_E5B_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGalileoL5()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x09);  // CFG-SIGNAL-GAL_E5A_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableBeiDou(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x22);  // CFG-SIGNAL-BDS_ENA
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableBeiDouL1()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x0D);  // CFG-SIGNAL-BDS_B1_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableBeiDouL2()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x0E);  // CFG-SIGNAL-BDS_B2_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableBeiDouL5()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x28);  // CFG-SIGNAL-BDS_B2A_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableQzss(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x24);  // CFG-SIGNAL-QZSS_ENA
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableQzssL1()
{
  CfgValSet<uint8_t, 2> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x12);  // CFG-SIGNAL-QZSS_L1CA_ENA
  cfg.data[0].value = true;

  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x14);  // CFG-SIGNAL-QZSS_L1S_ENA
  cfg.data[1].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableQzssL2()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x15);  // CFG-SIGNAL-QZSS_L2C_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableQzssL5()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x17);  // CFG-SIGNAL-QZSS_L5_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGlonass(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x25);  // CFG-SIGNAL-GLO_ENA
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGlonassL1()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x18);  // CFG-SIGNAL-GLO_L1_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableGlonassL2()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x1A);  // CFG-SIGNAL-GLO_L2_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableNavIc(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x26);  // CFG-SIGNAL-NAVIC_ENA
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::enableNavIcL5()
{
  CfgValSet<uint8_t, 1> cfg;

  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x1D);  // CFG-SIGNAL-NAVIC_L5_ENA
  cfg.data[0].value = true;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

ZEDF9P::CheckSum ZEDF9P::computeChecksum(const uint8_t* message, size_t checksum_pos)
{
  CheckSum ck;
  ck.CK_A = ck.CK_B = 0;

  for (size_t i = kUbxSyncLength; i < checksum_pos; ++i) {
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
}  // namespace ublox
