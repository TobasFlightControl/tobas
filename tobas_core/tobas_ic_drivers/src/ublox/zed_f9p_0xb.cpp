#include <cstring>
#include <cassert>

#include "../include/tobas_ic_drivers/ublox/zed_f9p_0xb.hpp"

#define NOT_IMPLEMENTED "Not implemented."
#define NOT_RECEIVABLE "Not receivable."

using namespace std;
using namespace chrono;

namespace ublox
{
ZEDF9P0xB::ZEDF9P0xB() : rate_(kReqInterval)
{
}

bool ZEDF9P0xB::initialize(const char* uart_device)
{
  // Initialize UART device
  if (!uart_.initialize(uart_device)) {
    cerr << "Uart initialization failed." << endl;
    return false;
  }

  if (!uart_.setBaudRate(kDefaultUartBaudRate)) {
    cerr << "Uart baudrate setting failed." << endl;
    return false;
  }

  if (!uart_.setDataBits(kUartdataBits)) {
    cerr << "Uart data bits setting failed." << endl;
    return false;
  }

  if (!uart_.setSingleStopBit()){
    cerr << "Uart single stop bit setting failed." << endl;
    return false;
  }

  if (!uart_.disableParity()){
    cerr << "Uart parity disabling failed." << endl;
    return false;
  }

  if (!uart_.disableHungupClose()){
    cerr << "Uart hungupClose disabling failed." << endl;
    return false;
  }

  return true;
}

bool ZEDF9P0xB::update(bool nonblock)
{
  scanner_.reset();

  if (nonblock) {
    // スタートバイトを確認
    if (!uart_.receive(rx_buf_, 1)) {
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
  while (scanner_.state() != UBXScanner::Done){
    if (uart_.receive(rx_buf_, 1)) {
        return false;
    }
    if (!scanner_.update(rx_buf_[0])) {
        return false;
    }

    // UARTリクエストの間隔が短すぎると正しくデータが取得できないのではないか？
    rate_.sleep();
  }

  if (!verifyMessage()) {
    return false;
  }

  return true;
}

bool ZEDF9P0xB::enableMsg(ubx_class_t cls, uint8_t id, bool enable)
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
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x66);  // CFG-MSGOUT-UBX_NAV_CLOCK_UART1
          break;
        case NAV_COV:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x84);  // CFG-MSGOUT-UBX_NAV_COV_UART1
          break;
        case NAV_DOP:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x39);  // CFG-MSGOUT-UBX_NAV_DOP_UART1
          break;
        case NAV_EOE:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x60);  // CFG-MSGOUT-UBX_NAV_EOE_UART1
          break;
        case NAV_GEOFENCE:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0xA2);  // CFG-MSGOUT-UBX_NAV_GEOFENCE_UART1
          break;
        case NAV_HPPOSECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x2F);  // CFG-MSGOUT-UBX_NAV_HPPOSECEF_UART1
          break;
        case NAV_HPPOSLLH:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x34);  // CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART1
          break;
        case NAV_ODO:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x7F);  // CFG-MSGOUT-UBX_NAV_ODO_UART1
          break;
        case NAV_ORB:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x11);  // CFG-MSGOUT-UBX_NAV_ORB_UART1
          break;
        case NAV_PL:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x16);  // CFG-MSGOUT-UBX_NAV_PL_UART1
          break;
        case NAV_POSECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x25);  // CFG-MSGOUT-UBX_NAV_POSECEF_UART1
          break;
        case NAV_POSLLH:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x2A);  // CFG-MSGOUT-UBX_NAV_POSLLH_UART1
          break;
        case NAV_PVT:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x07);  // CFG-MSGOUT-UBX_NAV_PVT_UART1
          break;
        case NAV_RELPOSNED:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x8E);  // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART1
          break;
        case NAV_RESETODO:
          cerr << NOT_RECEIVABLE << endl;
          return false;
        case NAV_SAT:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x16);  // CFG-MSGOUT-UBX_NAV_SAT_UART1
          break;
        case NAV_SBAS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x6B);  // CFG-MSGOUT-UBX_NAV_SBAS_UART1
          break;
        case NAV_SIG:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x46);  // CFG-MSGOUT-UBX_NAV_SIG_UART1
          break;
        case NAV_SLAS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x37);  // CFG-MSGOUT-UBX_NAV_SLAS_UART1
          break;
        case NAV_STATUS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x1B);  // CFG-MSGOUT-UBX_NAV_STATUS_UART1
          break;
        case NAV_SVIN:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x89);  // CFG-MSGOUT-UBX_NAV_SVIN_UART1
          break;
        case NAV_TIMEBDS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x52);  // CFG-MSGOUT-UBX_NAV_TIMEBDS_UART
          break;
        case NAV_TIMEGAL:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x57);  // CFG-MSGOUT-UBX_NAV_TIMEGAL_UART1
          break;
        case NAV_TIMEGLO:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x4d);  // CFG-MSGOUT-UBX_NAV_TIMEGLO_UART1
          break;
        case NAV_TIMEGPS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x48);  // CFG-MSGOUT-UBX_NAV_TIMEGPS_UART1
          break;
        case NAV_TIMELS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x61);  // CFG-MSGOUT-UBX_NAV_TIMELS_UART1
          break;
        case NAV_TIMEQZSS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x87);  // CFG-MSGOUT-UBX_NAV_TIMEQZSS_UART1
          break;
        case NAV_TIMEUTC:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x5C);  // CFG-MSGOUT-UBX_NAV_TIMEUTC_UART1
          break;
        case NAV_VELECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x3E);  // CFG-MSGOUT-UBX_NAV_VELECEF_UART1
          break;
        case NAV_VELNED:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x43);  // CFG-MSGOUT-UBX_NAV_VELNED_UART1
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

bool ZEDF9P0xB::configureDynamicsModel(dynamics_model_t model)
{
  CfgValSet<uint8_t, 1> cfg;

  // CFG-NAVSPG-DYNMODEL
  cfg.data[0].key = configKeyID(ONE_BYTE, CFG_NAVSPG, 0x21);
  cfg.data[0].value = model;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::configureMeasurementRate(uint16_t period_ms)
{
  CfgValSet<uint16_t, 1> cfg;

  // CFG-RATE-MEAS
  cfg.data[0].key = configKeyID(TWO_BYTES, CFG_RATE, 0x01);
  cfg.data[0].value = period_ms;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::configureBaudRate(uint32_t baudrate)
{
    CfgValSet<uint32_t, 1> cfg;

    // CFG-UART1-BAUDRATE
    cfg.data[0].key = configKeyID(FOUR_BYTES, CFG_UART1, 0x01);
    cfg.data[0].value = baudrate;

    if(!configure(CFG_VALSET, &cfg, sizeof(cfg))) {
      return false;
    }

    if (!uart_.setBaudRate(baudrate)) {
      cerr << "uart baudrate set failed" << endl;
      return false;
    }

    tobas_std::Rate baudrate_change_lag(kWaitForBaudRateChange);
    baudrate_change_lag.sleep();
    return true;
}

bool ZEDF9P0xB::enableGPS(bool enable)
{
  CfgValSet<uint8_t, 3> cfg;

  // CFG-SIGNAL-GPS_ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x1F);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-GPS_L1CA_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x01);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-GPS_L5_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x04);
  cfg.data[2].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::enableSBAS(bool enable)
{
  CfgValSet<uint8_t, 2> cfg;

  // CFG-SIGNAL-SBAS_ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x20);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-SBAS_L1CA_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x05);
  cfg.data[1].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::enableGalileo(bool enable)
{
  CfgValSet<uint8_t, 3> cfg;

  // CFG-SIGNAL-GAL_ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x21);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-GAL_E1_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x07);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-GAL_EA5_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x09);
  cfg.data[2].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::enableBeiDou(bool enable)
{
  CfgValSet<uint8_t, 3> cfg;

  // CFG-SIGNAL-BDS_ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x22);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-BDS_B1_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x0D);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-BDS_B2A_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x28);
  cfg.data[2].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::enableQZSS(bool enable)
{
  CfgValSet<uint8_t, 4> cfg;

  // CFG-SIGNAL-QZSS_ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x24);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-QZSS_L1CA_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x12);
  cfg.data[1].value = enable;

  // CFG-SIGNAL-QZSS_L1S_ENA
  cfg.data[2].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x14);
  cfg.data[2].value = enable;

  // CFG-SIGNAL-QZSS_L5_ENA
  cfg.data[3].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x17);
  cfg.data[3].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::enableGLONASS(bool enable)
{
  CfgValSet<uint8_t, 2> cfg;

  // CFG-SIGNAL-GLO_ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x25);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-GLO_L1_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x18);
  cfg.data[1].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::enableNavIC(bool enable)
{
  CfgValSet<uint8_t, 2> cfg;

  // CFG-SIGNAL-NAVIC_ENA
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x26);
  cfg.data[0].value = enable;

  // CFG-SIGNAL-NAVIC_L5_ENA
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_SIGNAL, 0x1D);
  cfg.data[1].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::enableProtocol(cfg_protocol_t prot, bool enable)
{
  CfgValSet<uint8_t, 2> cfg;

  // CFG-UART1INPROT-XXX
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_UART1INPROT, prot);
  cfg.data[0].value = enable;

  // CFG-UART1OUTPROT-XXX
  cfg.data[1].key = configKeyID(ONE_BIT, CFG_UART1OUTPROT, prot);
  cfg.data[1].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::setAntennaLength(uint8_t length_m)
{
  CfgValSet<uint16_t, 1> cfg;

  // CFG-TP-ANT_CABLEDELAY
  cfg.data[0].key = configKeyID(TWO_BYTES, CFG_RATE, 0x01);
  cfg.data[0].value = length_m * kRG174CableDelay;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::enableUSB(bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  // CFG-USB-ENABLED
  cfg.data[0].key = configKeyID(ONE_BIT, CFG_USB, 0x01);
  cfg.data[0].value = enable;

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P0xB::sendMessage(ubx_class_t cls, uint8_t id, const void* msg, uint16_t size)
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

  if (!uart_.send(tx_buf_, message_length)) {
    return false;
  }

  return true;
}

bool ZEDF9P0xB::waitForAcknowledge(ubx_class_t cls, uint8_t id)
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

bool ZEDF9P0xB::configure(ubx_cfg_id_t cfg_id, const void* msg, uint16_t size)
{
  if (!sendMessage(CLASS_CFG, cfg_id, msg, size)) {
    return false;
  }

  return waitForAcknowledge(CLASS_CFG, cfg_id);
}

bool ZEDF9P0xB::verifyMessage() const
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

ZEDF9P0xB::CheckSum ZEDF9P0xB::computeChecksum(const uint8_t* message, size_t checksum_pos)
{
  CheckSum ck;
  ck.CK_A = ck.CK_B = 0;

  for (size_t i = kUbxSyncLength; i < checksum_pos; ++i) {
    ck.CK_A += message[i];
    ck.CK_B += ck.CK_A;
  }

  return ck;
}

size_t ZEDF9P0xB::spliceMemory(uint8_t* dest, const void* src, size_t size, size_t dest_offset)
{
  memmove(dest + dest_offset, src, size);
  return dest_offset + size;
}

uint32_t ZEDF9P0xB::configKeyID(cfg_size_t size, cfg_group_t group, uint8_t id)
{
  return (size << 28) | (group << 16) | id;
}
}  // namespace ublox
