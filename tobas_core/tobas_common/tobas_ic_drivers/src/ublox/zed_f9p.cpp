// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/zed_f9p.hpp"

#include <cassert>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas
{
namespace ublox
{
ZEDF9P::ZEDF9P(std::unique_ptr<UbxTransport> _transport) : transport_(std::move(_transport)), scan_rate_(50us)
{
  assert(transport_);
}

bool ZEDF9P::initialize()
{
  return transport_->initialize();
}

bool ZEDF9P::update(bool _blocking)
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

bool ZEDF9P::enableSpiMessage(UbxClass _cls, uint8_t _id, bool _enable)
{
  constexpr char kNotImplemented[] = "Not implemented.";
  constexpr char kNotReceivable[] = "Not receivable.";

  CfgValSet<uint8_t, 1> cfg;

  switch (_cls) {
    case CLASS_ACK: {
      std::cerr << kNotReceivable << '\n';
      return false;
    }
    case CLASS_CFG: {
      std::cerr << kNotReceivable << '\n';
      return false;
    }
    case CLASS_INF: {
      std::cerr << kNotImplemented << '\n';  // TODO
      return false;
    }
    case CLASS_LOG: {
      std::cerr << kNotImplemented << '\n';  // TODO
      return false;
    }
    case CLASS_MGA: {
      std::cerr << kNotImplemented << '\n';  // TODO
      return false;
    }
    case CLASS_MON: {
      std::cerr << kNotImplemented << '\n';  // TODO
      return false;
    }
    case CLASS_NAV: {
      switch (_id) {
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
          std::cerr << kNotReceivable << '\n';
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
          std::cerr << kNotImplemented << '\n';  // TODO
          return false;
      }
      break;
    }
    case CLASS_NAV2: {
      std::cerr << kNotImplemented << '\n';  // TODO
      return false;
    }
    case CLASS_RXM: {
      std::cerr << kNotImplemented << '\n';  // TODO
      return false;
    }
    case CLASS_SEC: {
      std::cerr << kNotImplemented << '\n';  // TODO
      return false;
    }
    case CLASS_TIM: {
      std::cerr << kNotImplemented << '\n';  // TODO
      return false;
    }
    case CLASS_UPD: {
      std::cerr << kNotImplemented << '\n';  // TODO
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

bool ZEDF9P::configureDynamicsModel(DynamicsModel _model)
{
  return cfgValSetSingle<uint8_t>(ONE_BYTE, CFG_NAVSPG, 0x21, _model);  // CFG-NAVSPG-DYNMODEL
}

bool ZEDF9P::configureMeasurementRate(uint16_t _period_ms)
{
  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_RATE, 0x01, _period_ms);  // CFG-RATE-MEAS
}

bool ZEDF9P::enableGps()
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

bool ZEDF9P::disableGps()
{
  return enableGps(false);
}

bool ZEDF9P::enableSbas()
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

bool ZEDF9P::disableSbas()
{
  return enableGps(false);
}

bool ZEDF9P::enableGalileo()
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

bool ZEDF9P::disableGalileo()
{
  return enableGalileo(false);
}

bool ZEDF9P::enableBeiDou()
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

bool ZEDF9P::disableBeiDou()
{
  return enableBeiDou(false);
}

bool ZEDF9P::enableQzss()
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

bool ZEDF9P::disableQzss()
{
  return enableQzss(false);
}

bool ZEDF9P::enableGlonass()
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

bool ZEDF9P::disableGlonass()
{
  return enableGlonass(false);
}

bool ZEDF9P::enableNavIc()
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

bool ZEDF9P::disableNavIc()
{
  return enableNavIc(false);
}

bool ZEDF9P::enableSpiProtocol_UBX(bool _enable_input, bool _enable_output)
{
  return enableSpiInputProtocol(UBX, _enable_input) && enableSpiOutputProtocol(UBX, _enable_output);
}

bool ZEDF9P::enableSpiProtocol_NMEA(bool _enable_input, bool _enable_output)
{
  return enableSpiInputProtocol(NMEA, _enable_input) && enableSpiOutputProtocol(NMEA, _enable_output);
}

bool ZEDF9P::enableSpiProtocol_RTCM3X(bool _enable_input, bool _enable_output)
{
  return enableSpiInputProtocol(RTCM3X, _enable_input) && enableSpiOutputProtocol(RTCM3X, _enable_output);
}

bool ZEDF9P::enableSpiProtocol_SPARTN(bool _enable_input)
{
  return enableSpiInputProtocol(SPARTN, _enable_input);
}

bool ZEDF9P::setAntennaLength(uint8_t _length_m)
{
  constexpr uint8_t kRG174CableDelay = 5;  // [ns/m] Coaxial cable delay.
  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_TP, 0x01, _length_m * kRG174CableDelay);  // CFG-TP-ANT_CABLEDELAY
}

bool ZEDF9P::enableUsb(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_USB, 0x01, _enable);  // CFG-USB-ENABLED
}

bool ZEDF9P::sendMessage(UbxClass _cls, uint8_t _id, const void* _msg, uint16_t _size)
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

bool ZEDF9P::waitForAcknowledge(UbxClass _cls, uint8_t _id)
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

bool ZEDF9P::configure(UbxCfgId _cfg_id, const void* _msg, uint16_t _size)
{
  return sendMessage(CLASS_CFG, _cfg_id, _msg, _size) && waitForAcknowledge(CLASS_CFG, _cfg_id);
}

bool ZEDF9P::verifyMessage() const
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

bool ZEDF9P::enableGps(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1F, _enable);  // CFG-SIGNAL-GPS_ENA
}

bool ZEDF9P::enableGpsL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x01, true);  // CFG-SIGNAL-GPS_L1CA_ENA
}

bool ZEDF9P::enableGpsL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x03, true);  // CFG-SIGNAL-GPS_L2C_ENA
}

bool ZEDF9P::enableGpsL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x04, true);  // CFG-SIGNAL-GPS_L5_ENA
}

bool ZEDF9P::enableSbas(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x20, _enable);  // CFG-SIGNAL-SBAS_ENA
}

bool ZEDF9P::enableSbasL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x05, true);  // CFG-SIGNAL-SBAS_L1CA_ENA
}

bool ZEDF9P::enableGalileo(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x21, _enable);  // CFG-SIGNAL-GAL_ENA
}

bool ZEDF9P::enableGalileoL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x07, true);  // CFG-SIGNAL-GAL_E1_ENA
}

bool ZEDF9P::enableGalileoL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0A, true);  // CFG-SIGNAL-GAL_E5B_ENA
}

bool ZEDF9P::enableGalileoL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x09, true);  // CFG-SIGNAL-GAL_E5A_ENA
}

bool ZEDF9P::enableBeiDou(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x22, _enable);  // CFG-SIGNAL-BDS_ENA
}

bool ZEDF9P::enableBeiDouL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0D, true);  // CFG-SIGNAL-BDS_B1_ENA
}

bool ZEDF9P::enableBeiDouL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0E, true);  // CFG-SIGNAL-BDS_B2_ENA
}

bool ZEDF9P::enableBeiDouL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x28, true);  // CFG-SIGNAL-BDS_B2A_ENA
}

bool ZEDF9P::enableQzss(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x24, _enable);  // CFG-SIGNAL-QZSS_ENA
}

bool ZEDF9P::enableQzssL1()
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

bool ZEDF9P::enableQzssL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x15, true);  // CFG-SIGNAL-QZSS_L2C_ENA
}

bool ZEDF9P::enableQzssL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x17, true);  // CFG-SIGNAL-QZSS_L5_ENA
}

bool ZEDF9P::enableGlonass(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x25, _enable);  // CFG-SIGNAL-GLO_ENA
}

bool ZEDF9P::enableGlonassL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x18, true);  // CFG-SIGNAL-GLO_L1_ENA
}

bool ZEDF9P::enableGlonassL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1A, true);  // CFG-SIGNAL-GLO_L2_ENA
}

bool ZEDF9P::enableNavIc(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x26, _enable);  // CFG-SIGNAL-NAVIC_ENA
}

bool ZEDF9P::enableNavIcL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1D, true);  // CFG-SIGNAL-NAVIC_L5_ENA
}

bool ZEDF9P::enableSpiInputProtocol(CfgProtocol _prot, bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SPIINPROT, _prot, _enable);  // CFG-SPIINPROT-XXX
}

bool ZEDF9P::enableSpiOutputProtocol(CfgProtocol _prot, bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SPIOUTPROT, _prot, _enable);  // CFG-SPIOUTPROT-XXX
}

ZEDF9P::CheckSum ZEDF9P::computeChecksum(const uint8_t* _message, size_t _checksum_pos)
{
  CheckSum ck;
  ck.CK_A = ck.CK_B = 0;

  for (size_t i = kUbxSyncLength; i < _checksum_pos; ++i) {
    ck.CK_A += _message[i];
    ck.CK_B += ck.CK_A;
  }

  return ck;
}

size_t ZEDF9P::spliceMemory(uint8_t* _dest, const void* _src, size_t _size, size_t _dest_offset)
{
  std::memmove(_dest + _dest_offset, _src, _size);
  return _dest_offset + _size;
}

uint32_t ZEDF9P::configKeyID(CfgSize _size, CfgGroup _group, uint8_t _id)
{
  return (_size << 28) | (_group << 16) | _id;
}
}  // namespace ublox
}  // namespace tobas
