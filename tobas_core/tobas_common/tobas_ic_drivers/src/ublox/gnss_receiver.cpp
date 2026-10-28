// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/gnss_receiver.hpp"

#include <cassert>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas::ublox
{
GnssReceiver::GnssReceiver(std::unique_ptr<UbxTransport> _transport)
  : transport_(std::move(_transport)), scan_rate_(50us)
{
  assert(transport_);
}

bool GnssReceiver::initialize()
{
  return transport_->initialize();
}

bool GnssReceiver::update(bool _blocking)
{
  scanner_.reset();

  if (!_blocking) {
    // Check the start byte.
    const auto first_byte = transport_->receiveByte();
    if (!first_byte) {
      return false;
    }
    scanner_.update(*first_byte);

    // Return if no data has arrived.
    if (scanner_.state() == UbxScanner::kSync1) {
      return false;
    }
  }

  // Scan one message.
  scan_rate_.start();
  while (scanner_.state() != UbxScanner::kDone) {
    const auto data = transport_->receiveByte();
    if (!data) {
      return false;
    }
    scanner_.update(*data);
    scan_rate_.sleep();
  }

  if (!verifyMessage()) {
    return false;
  }

  return true;
}

bool GnssReceiver::enableMessage(UbxClass _cls, uint8_t _id, bool _enable)
{
  constexpr char kNotImplemented[] = "Not implemented.\n";
  constexpr char kNotReceivable[] = "Not receivable.\n";

  CfgValSet<uint8_t, 1> cfg;

  switch (_cls) {
    case CLASS_ACK: {
      std::cerr << kNotReceivable;
      return false;
    }
    case CLASS_CFG: {
      std::cerr << kNotReceivable;
      return false;
    }
    case CLASS_INF: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }
    case CLASS_LOG: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }
    case CLASS_MGA: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }
    case CLASS_MON: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }

    case CLASS_NAV: {
      struct MsgOutIds
      {
        uint16_t spi;
        uint16_t uart1;
      };

      MsgOutIds msgout_ids{};

      switch (_id) {
        case NAV_CLOCK:
          msgout_ids = { 0x069, 0x066 };  // CFG-MSGOUT-UBX_NAV_CLOCK: SPI, UART1
          break;
        case NAV_COV:
          msgout_ids = { 0x087, 0x084 };  // CFG-MSGOUT-UBX_NAV_COV: SPI, UART1
          break;
        case NAV_DOP:
          msgout_ids = { 0x03C, 0x039 };  // CFG-MSGOUT-UBX_NAV_DOP: SPI, UART1
          break;
        case NAV_EOE:
          msgout_ids = { 0x163, 0x160 };  // CFG-MSGOUT-UBX_NAV_EOE: SPI, UART1
          break;
        case NAV_GEOFENCE:
          msgout_ids = { 0x0A5, 0x0A2 };  // CFG-MSGOUT-UBX_NAV_GEOFENCE: SPI, UART1
          break;
        case NAV_HPPOSECEF:
          msgout_ids = { 0x032, 0x02F };  // CFG-MSGOUT-UBX_NAV_HPPOSECEF: SPI, UART1
          break;
        case NAV_HPPOSLLH:
          msgout_ids = { 0x037, 0x034 };  // CFG-MSGOUT-UBX_NAV_HPPOSLLH: SPI, UART1
          break;
        case NAV_ODO:
          msgout_ids = { 0x082, 0x07F };  // CFG-MSGOUT-UBX_NAV_ODO: SPI, UART1
          break;
        case NAV_ORB:
          msgout_ids = { 0x014, 0x011 };  // CFG-MSGOUT-UBX_NAV_ORB: SPI, UART1
          break;
        case NAV_PL:
          msgout_ids = { 0x419, 0x416 };  // CFG-MSGOUT-UBX_NAV_PL: SPI, UART1
          break;
        case NAV_POSECEF:
          msgout_ids = { 0x028, 0x025 };  // CFG-MSGOUT-UBX_NAV_POSECEF: SPI, UART1
          break;
        case NAV_POSLLH:
          msgout_ids = { 0x02D, 0x02A };  // CFG-MSGOUT-UBX_NAV_POSLLH: SPI, UART1
          break;
        case NAV_PVT:
          msgout_ids = { 0x00A, 0x007 };  // CFG-MSGOUT-UBX_NAV_PVT: SPI, UART1
          break;
        case NAV_RELPOSNED:
          msgout_ids = { 0x091, 0x08E };  // CFG-MSGOUT-UBX_NAV_RELPOSNED: SPI, UART1
          break;
        case NAV_RESETODO:
          std::cerr << kNotReceivable;
          return false;
        case NAV_SAT:
          msgout_ids = { 0x019, 0x016 };  // CFG-MSGOUT-UBX_NAV_SAT: SPI, UART1
          break;
        case NAV_SBAS:
          msgout_ids = { 0x06E, 0x06B };  // CFG-MSGOUT-UBX_NAV_SBAS: SPI, UART1
          break;
        case NAV_SIG:
          msgout_ids = { 0x349, 0x346 };  // CFG-MSGOUT-UBX_NAV_SIG: SPI, UART1
          break;
        case NAV_SLAS:
          msgout_ids = { 0x33A, 0x337 };  // CFG-MSGOUT-UBX_NAV_SLAS: SPI, UART1
          break;
        case NAV_STATUS:
          msgout_ids = { 0x01E, 0x01B };  // CFG-MSGOUT-UBX_NAV_STATUS: SPI, UART1
          break;
        case NAV_SVIN:
          msgout_ids = { 0x08C, 0x089 };  // CFG-MSGOUT-UBX_NAV_SVIN: SPI, UART1
          break;
        case NAV_TIMEBDS:
          msgout_ids = { 0x055, 0x052 };  // CFG-MSGOUT-UBX_NAV_TIMEBDS: SPI, UART1
          break;
        case NAV_TIMEGAL:
          msgout_ids = { 0x05A, 0x057 };  // CFG-MSGOUT-UBX_NAV_TIMEGAL: SPI, UART1
          break;
        case NAV_TIMEGLO:
          msgout_ids = { 0x050, 0x04D };  // CFG-MSGOUT-UBX_NAV_TIMEGLO: SPI, UART1
          break;
        case NAV_TIMEGPS:
          msgout_ids = { 0x04B, 0x048 };  // CFG-MSGOUT-UBX_NAV_TIMEGPS: SPI, UART1
          break;
        case NAV_TIMELS:
          msgout_ids = { 0x064, 0x061 };  // CFG-MSGOUT-UBX_NAV_TIMELS: SPI, UART1
          break;
        case NAV_TIMEQZSS:
          msgout_ids = { 0x38A, 0x387 };  // CFG-MSGOUT-UBX_NAV_TIMEQZSS: SPI, UART1
          break;
        case NAV_TIMEUTC:
          msgout_ids = { 0x05F, 0x05C };  // CFG-MSGOUT-UBX_NAV_TIMEUTC: SPI, UART1
          break;
        case NAV_VELECEF:
          msgout_ids = { 0x041, 0x03E };  // CFG-MSGOUT-UBX_NAV_VELECEF: SPI, UART1
          break;
        case NAV_VELNED:
          msgout_ids = { 0x046, 0x043 };  // CFG-MSGOUT-UBX_NAV_VELNED: SPI, UART1
          break;
        default:
          std::cerr << kNotImplemented;  // TODO
          return false;
      }

      uint16_t item_id;

      switch (transport_->interface()) {
        case UbxTransport::Interface::SPI:
          item_id = msgout_ids.spi;
          break;
        case UbxTransport::Interface::UART:
          item_id = msgout_ids.uart1;
          break;
        default:
          return false;
      }

      cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, item_id);
      break;
    }
    case CLASS_NAV2: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }
    case CLASS_RXM: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }
    case CLASS_SEC: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }
    case CLASS_TIM: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }
    case CLASS_UPD: {
      std::cerr << kNotImplemented;  // TODO
      return false;
    }
    default: {
      std::cerr << "Invalid UBX class type: " << std::hex << _cls << '\n';
      return false;
    }
  }

  cfg.data[0].value = _enable ? 1 : 0;  // Maximum rate if enabled.

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool GnssReceiver::configureDynamicsModel(DynamicsModel _model)
{
  return cfgValSetSingle<uint8_t>(ONE_BYTE, CFG_NAVSPG, 0x21, _model);  // CFG-NAVSPG-DYNMODEL
}

bool GnssReceiver::configureMeasurementRate(uint16_t _period_ms)
{
  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_RATE, 0x01, _period_ms);  // CFG-RATE-MEAS
}

bool GnssReceiver::enableGps()
{
  // Enable GPS.
  if (!enableGps(true)) {
    std::cerr << "Failed to enable GPS.\n";
    return false;
  }

  // Enable L1 band.
  if (!enableGpsL1()) {
    std::cerr << "Failed to enable GPS L1.\n";
    return false;
  }

  // Try to enable L2 band.
  if (enableGpsL2()) {
    std::cout << "GPS L1/L2 is enabled.\n";
    return true;
  }

  // Try to enable L5 band.
  if (enableGpsL5()) {
    std::cout << "GPS L1/L5 is enabled.\n";
    return true;
  }

  std::cerr << "Failed to enable either GPS L2 or L5 bands.\n";
  return false;
}

bool GnssReceiver::disableGps()
{
  return enableGps(false);
}

bool GnssReceiver::enableSbas()
{
  // Enable SBAS.
  if (!enableSbas(true)) {
    std::cerr << "Failed to enable SBAS.\n";
    return false;
  }

  // Enable L1 band.
  if (!enableSbasL1()) {
    std::cerr << "Failed to enable SBAS L1.\n";
    return false;
  }

  return true;
}

bool GnssReceiver::disableSbas()
{
  return enableGps(false);
}

bool GnssReceiver::enableGalileo()
{
  // Enable Galileo.
  if (!enableGalileo(true)) {
    std::cerr << "Failed to enable Galileo.\n";
    return false;
  }

  // Enable L1 band.
  if (!enableGalileoL1()) {
    std::cerr << "Failed to enable Galileo L1.\n";
    return false;
  }

  // Try to enable L2 band.
  if (enableGalileoL2()) {
    std::cout << "Galileo L1/L2 is enabled.\n";
    return true;
  }

  // Try to enable L5 band.
  if (enableGalileoL5()) {
    std::cout << "Galileo L1/L5 is enabled.\n";
    return true;
  }

  std::cerr << "Failed to enable either Galileo L2 or L5 bands.\n";
  return false;
}

bool GnssReceiver::disableGalileo()
{
  return enableGalileo(false);
}

bool GnssReceiver::enableBeiDou()
{
  // Enable BeiDou.
  if (!enableBeiDou(true)) {
    std::cerr << "Failed to enable BeiDou.\n";
    return false;
  }

  // Enable L1 band.
  if (!enableBeiDouL1()) {
    std::cerr << "Failed to enable BeiDou L1.\n";
    return false;
  }

  // Try to enable L2 band.
  if (enableBeiDouL2()) {
    std::cout << "BeiDou L1/L2 is enabled.\n";
    return true;
  }

  // Try to enable L5 band.
  if (enableBeiDouL5()) {
    std::cout << "BeiDou L1/L5 is enabled.\n";
    return true;
  }

  std::cerr << "Failed to enable either BeiDou L2 or L5 bands.\n";
  return false;
}

bool GnssReceiver::disableBeiDou()
{
  return enableBeiDou(false);
}

bool GnssReceiver::enableQzss()
{
  // Enable QZSS.
  if (!enableQzss(true)) {
    std::cerr << "Failed to enable QZSS.\n";
    return false;
  }

  // Enable L1 band.
  if (!enableQzssL1()) {
    std::cerr << "Failed to enable QZSS L1.\n";
    return false;
  }

  // Try to enable L2 band.
  if (enableQzssL2()) {
    std::cout << "QZSS L1/L2 is enabled.\n";
    return true;
  }

  // Try to enable L5 band.
  if (enableQzssL5()) {
    std::cout << "QZSS L1/L5 is enabled.\n";
    return true;
  }

  std::cerr << "Failed to enable either QZSS L2 or L5 bands.\n";
  return false;
}

bool GnssReceiver::disableQzss()
{
  return enableQzss(false);
}

bool GnssReceiver::enableGlonass()
{
  // Enable GLONASS.
  if (!enableGlonass(true)) {
    std::cerr << "Failed to enable GLONASS.\n";
    return false;
  }

  // Enable L1 band.
  if (!enableGlonassL1()) {
    std::cerr << "Failed to enable GLONASS L1.\n";
    return false;
  }

  // Try to enable L2 band.
  if (enableGlonassL2()) {
    std::cout << "GLONASS L1/L2 is enabled.\n";
    return true;
  }

  std::cout << "GLONASS L1 is enabled.\n";
  return true;
}

bool GnssReceiver::disableGlonass()
{
  return enableGlonass(false);
}

bool GnssReceiver::enableNavIc()
{
  // Enable NavIC.
  if (!enableNavIc(true)) {
    std::cerr << "Failed to enable NavIC.\n";
    return false;
  }

  // Enable L5 band.
  if (!enableNavIcL5()) {
    std::cerr << "Failed to enable NavIC L5.\n";
    return false;
  }

  std::cout << "NavIC L5 is enabled.\n";
  return false;
}

bool GnssReceiver::disableNavIc()
{
  return enableNavIc(false);
}

bool GnssReceiver::enableProtocol_UBX(bool _enable_input, bool _enable_output)
{
  return enableInputProtocol(UBX, _enable_input) && enableOutputProtocol(UBX, _enable_output);
}

bool GnssReceiver::enableProtocol_NMEA(bool _enable_input, bool _enable_output)
{
  return enableInputProtocol(NMEA, _enable_input) && enableOutputProtocol(NMEA, _enable_output);
}

bool GnssReceiver::enableProtocol_RTCM3X(bool _enable_input, bool _enable_output)
{
  return enableInputProtocol(RTCM3X, _enable_input) && enableOutputProtocol(RTCM3X, _enable_output);
}

bool GnssReceiver::enableProtocol_SPARTN(bool _enable_input)
{
  return enableInputProtocol(SPARTN, _enable_input);
}

bool GnssReceiver::setAntennaLength(uint8_t _length_m)
{
  constexpr uint8_t kRG174CableDelay = 5;  // [ns/m] Coaxial cable delay.
  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_TP, 0x01, _length_m * kRG174CableDelay);  // CFG-TP-ANT_CABLEDELAY
}

bool GnssReceiver::enableUsb(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_USB, 0x01, _enable);  // CFG-USB-ENABLED
}

bool GnssReceiver::sendMessage(UbxClass _cls, uint8_t _id, const void* _msg, uint16_t _size)
{
  // Allocate space for the header, payload, and checksum of this message.
  std::vector<uint8_t> tx_buf(sizeof(UbxHeader) + _size + sizeof(CheckSum));

  UbxHeader header;
  header.sync1 = kUbxSync1;
  header.sync2 = kUbxSync2;
  header.cls = _cls;
  header.id = _id;
  header.length = _size;

  const auto payload_pos = spliceMemory(tx_buf.data(), &header, sizeof(UbxHeader), 0);
  const auto checksum_pos = spliceMemory(tx_buf.data(), _msg, _size, payload_pos);

  const auto ck = computeChecksum(tx_buf.data(), checksum_pos);
  const auto message_length = spliceMemory(tx_buf.data(), &ck, sizeof(CheckSum), checksum_pos);

  return transport_->send(tx_buf.data(), message_length);
}

bool GnssReceiver::waitForAcknowledge(UbxClass _cls, uint8_t _id)
{
  payload::ACK_ACK ack;
  payload::ACK_NAK nak;

  const auto cls_str = std::to_string(static_cast<int>(_cls));
  const auto id_str = std::to_string(static_cast<int>(_id));

  constexpr auto kWaitForGnssAck = 1s;
  const auto deadline = ch::steady_clock::now() + kWaitForGnssAck;

  while (ch::steady_clock::now() < deadline) {
    if (!update(true)) {
      return false;
    }

    if (latestClass() != CLASS_ACK) {
      continue;
    }

    switch (latestId()) {
      case ACK_ACK:
        ack.decode(payload());

        if (ack.clsID == _cls && ack.msgID == _id) {
          return true;
        }
        else {
          std::cerr << "An acknowledment message for an unspecified message is received.\n";
          return false;
        }

        break;

      case ACK_NAK:
        nak.decode(payload());

        if (nak.clsID == _cls && nak.msgID == _id) {
          std::cerr << "Configuration was rejected: (class, id) = (" << cls_str << ", " << id_str << ")\n";
          return false;
        }
        else {
          std::cerr << "A non-acknowledment message for an unspecified message is received.\n";
          return false;
        }

        break;

      default:
        std::cerr << "Unexpected ACK ID: " << static_cast<int>(latestId()) << '\n';
        break;
    }
  }

  std::cerr << "Acknowledment message not received: (class, id) = (" << cls_str << ", " << id_str << ")\n";
  return false;
}

bool GnssReceiver::configure(UbxCfgId _cfg_id, const void* _msg, uint16_t _size)
{
  return sendMessage(CLASS_CFG, _cfg_id, _msg, _size) && waitForAcknowledge(CLASS_CFG, _cfg_id);
}

bool GnssReceiver::verifyMessage() const
{
  // Sync chars
  if (*scanner_.getSync1() != kUbxSync1 || *scanner_.getSync2() != kUbxSync2) {
    std::cerr << "The current message is not UBX format.\n";
    return false;
  }

  // Checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (auto x = scanner_.getClass(); x < scanner_.getChecksumA(); ++x) {
    CK_A += *x;
    CK_B += CK_A;
  }
  if (CK_A != *scanner_.getChecksumA() || CK_B != *scanner_.getChecksumB()) {
    std::cerr << "Checksum failed.\n";
    return false;
  }

  return true;
}

bool GnssReceiver::enableGps(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1F, _enable);  // CFG-SIGNAL-GPS_ENA
}

bool GnssReceiver::enableGpsL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x01, true);  // CFG-SIGNAL-GPS_L1CA_ENA
}

bool GnssReceiver::enableGpsL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x03, true);  // CFG-SIGNAL-GPS_L2C_ENA
}

bool GnssReceiver::enableGpsL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x04, true);  // CFG-SIGNAL-GPS_L5_ENA
}

bool GnssReceiver::enableSbas(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x20, _enable);  // CFG-SIGNAL-SBAS_ENA
}

bool GnssReceiver::enableSbasL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x05, true);  // CFG-SIGNAL-SBAS_L1CA_ENA
}

bool GnssReceiver::enableGalileo(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x21, _enable);  // CFG-SIGNAL-GAL_ENA
}

bool GnssReceiver::enableGalileoL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x07, true);  // CFG-SIGNAL-GAL_E1_ENA
}

bool GnssReceiver::enableGalileoL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0A, true);  // CFG-SIGNAL-GAL_E5B_ENA
}

bool GnssReceiver::enableGalileoL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x09, true);  // CFG-SIGNAL-GAL_E5A_ENA
}

bool GnssReceiver::enableBeiDou(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x22, _enable);  // CFG-SIGNAL-BDS_ENA
}

bool GnssReceiver::enableBeiDouL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0D, true);  // CFG-SIGNAL-BDS_B1_ENA
}

bool GnssReceiver::enableBeiDouL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0E, true);  // CFG-SIGNAL-BDS_B2_ENA
}

bool GnssReceiver::enableBeiDouL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x28, true);  // CFG-SIGNAL-BDS_B2A_ENA
}

bool GnssReceiver::enableQzss(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x24, _enable);  // CFG-SIGNAL-QZSS_ENA
}

bool GnssReceiver::enableQzssL1()
{
  // CFG-SIGNAL-QZSS_L1CA_ENA
  if (!cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x12, true)) {
    return false;
  }

  // CFG-SIGNAL-QZSS_L1S_ENA
  if (!cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x14, true)) {
    return false;
  }

  return true;
}

bool GnssReceiver::enableQzssL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x15, true);  // CFG-SIGNAL-QZSS_L2C_ENA
}

bool GnssReceiver::enableQzssL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x17, true);  // CFG-SIGNAL-QZSS_L5_ENA
}

bool GnssReceiver::enableGlonass(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x25, _enable);  // CFG-SIGNAL-GLO_ENA
}

bool GnssReceiver::enableGlonassL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x18, true);  // CFG-SIGNAL-GLO_L1_ENA
}

bool GnssReceiver::enableGlonassL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1A, true);  // CFG-SIGNAL-GLO_L2_ENA
}

bool GnssReceiver::enableNavIc(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x26, _enable);  // CFG-SIGNAL-NAVIC_ENA
}

bool GnssReceiver::enableNavIcL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1D, true);  // CFG-SIGNAL-NAVIC_L5_ENA
}

bool GnssReceiver::enableInputProtocol(CfgProtocol _prot, bool _enable)
{
  CfgGroup group;

  switch (transport_->interface()) {
    case UbxTransport::Interface::SPI:
      group = CFG_SPIINPROT;
      break;
    case UbxTransport::Interface::UART:
      group = CFG_UART1INPROT;
      break;
    default:
      return false;
  }

  return cfgValSetSingle<uint8_t>(ONE_BIT, group, _prot, _enable);
}

bool GnssReceiver::enableOutputProtocol(CfgProtocol _prot, bool _enable)
{
  CfgGroup group;

  switch (transport_->interface()) {
    case UbxTransport::Interface::SPI:
      group = CFG_SPIOUTPROT;
      break;
    case UbxTransport::Interface::UART:
      group = CFG_UART1OUTPROT;
      break;
    default:
      return false;
  }

  return cfgValSetSingle<uint8_t>(ONE_BIT, group, _prot, _enable);
}

GnssReceiver::CheckSum GnssReceiver::computeChecksum(const uint8_t* _message, size_t _checksum_pos)
{
  CheckSum ck;
  ck.CK_A = ck.CK_B = 0;

  for (size_t i = kUbxSyncLength; i < _checksum_pos; ++i) {
    ck.CK_A += _message[i];
    ck.CK_B += ck.CK_A;
  }

  return ck;
}

size_t GnssReceiver::spliceMemory(uint8_t* _dest, const void* _src, size_t _size, size_t _dest_offset)
{
  std::memmove(_dest + _dest_offset, _src, _size);
  return _dest_offset + _size;
}

uint32_t GnssReceiver::configKeyID(CfgSize _size, CfgGroup _group, uint16_t _id)
{
  return (_size << 28) | (_group << 16) | (_id & 0x0FFF);
}
}  // namespace tobas::ublox
