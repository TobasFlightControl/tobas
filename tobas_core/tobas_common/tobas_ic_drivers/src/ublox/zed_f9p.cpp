// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/zed_f9p.hpp"

#include <cassert>
#include <cstring>
#include <memory>
#include <utility>

#include "tobas_ic_drivers/ublox/ubx_spi_transport.hpp"

#define NOT_IMPLEMENTED "Not implemented."
#define NOT_RECEIVABLE "Not receivable."

namespace ch = std::chrono;

namespace tobas
{
namespace ublox
{
ZEDF9P::ZEDF9P() : ZEDF9P(std::make_unique<UbxTransportSpi>())
{
}

ZEDF9P::ZEDF9P(std::unique_ptr<UbxTransport> _transport) : transport_(std::move(_transport))
{
  assert(transport_);
}

bool ZEDF9P::initialize(const char* _device)
{
  receive_rate_.reset();

  const auto interval = transport_->receiveByteInterval();
  if (interval.count() < 0) {
    return false;
  }

  if (!transport_->initialize(_device)) {
    return false;
  }

  if (interval.count() > 0) {
    receive_rate_.emplace(interval);
  }

  return true;
}

bool ZEDF9P::update(bool nonblock)
{
  scanner_.reset();

  if (nonblock) {
    // Check the start byte.
    const auto data = transport_->receiveByte();
    if (!data) {
      return false;
    }
    if (!scanner_.update(*data)) {
      return false;
    }

    // Return if no data has arrived.
    if (scanner_.state() == UbxScanner::kSync1) {
      return false;
    }
  }

  // Scan one message.
  if (receive_rate_) {
    receive_rate_->start();
  }
  while (scanner_.state() != UbxScanner::kDone) {
    const auto data = transport_->receiveByte();
    if (!data) {
      return false;
    }
    if (!scanner_.update(*data)) {
      return false;
    }
    if (receive_rate_) {
      receive_rate_->sleep();
    }
  }

  if (!verifyMessage()) {
    return false;
  }

  return true;
}

bool ZEDF9P::enableSpiMessage(UbxClass cls, uint8_t id, bool enable)
{
  CfgValSet<uint8_t, 1> cfg;

  switch (cls) {
    case CLASS_ACK: {
      std::cerr << NOT_RECEIVABLE << std::endl;
      return false;
    }
    case CLASS_CFG: {
      std::cerr << NOT_RECEIVABLE << std::endl;
      return false;
    }
    case CLASS_INF: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
      return false;
    }
    case CLASS_LOG: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
      return false;
    }
    case CLASS_MGA: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
      return false;
    }
    case CLASS_MON: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
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
          std::cerr << NOT_RECEIVABLE << std::endl;
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
          std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
          return false;
      }
      break;
    }
    case CLASS_NAV2: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
      return false;
    }
    case CLASS_RXM: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
      return false;
    }
    case CLASS_SEC: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
      return false;
    }
    case CLASS_TIM: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
      return false;
    }
    case CLASS_UPD: {
      std::cerr << NOT_IMPLEMENTED << std::endl;  // TODO
      return false;
    }
    default: {
      std::cerr << "Invalid UBX class type: " << std::hex << cls << std::endl;
      return false;
    }
  }

  cfg.data[0].value = enable ? 1 : 0;  // Maximum rate if enabled.

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool ZEDF9P::configureDynamicsModel(DynamicsModel model)
{
  return cfgValSetSingle<uint8_t>(ONE_BYTE, CFG_NAVSPG, 0x21, model);  // CFG-NAVSPG-DYNMODEL
}

bool ZEDF9P::configureMeasurementRate(uint16_t period_ms)
{
  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_RATE, 0x01, period_ms);  // CFG-RATE-MEAS
}

bool ZEDF9P::enableGps()
{
  // Enable GPS.
  if (!enableGps(true)) {
    std::cerr << "Failed to enable GPS." << std::endl;
    return false;
  }

  // Enable L1 band.
  if (!enableGpsL1()) {
    std::cerr << "Failed to enable GPS L1." << std::endl;
    return false;
  }

  // Try to enable L2 band.
  if (enableGpsL2()) {
    std::cout << "GPS L1/L2 is enabled." << std::endl;
    return true;
  }

  // Try to enable L5 band.
  if (enableGpsL5()) {
    std::cout << "GPS L1/L5 is enabled." << std::endl;
    return true;
  }

  std::cerr << "Failed to enable either GPS L2 or L5 bands." << std::endl;
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
    std::cerr << "Failed to enable SBAS." << std::endl;
    return false;
  }

  // Enable L1 band.
  if (!enableSbasL1()) {
    std::cerr << "Failed to enable SBAS L1." << std::endl;
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
    std::cerr << "Failed to enable Galileo." << std::endl;
    return false;
  }

  // Enable L1 band.
  if (!enableGalileoL1()) {
    std::cerr << "Failed to enable Galileo L1." << std::endl;
    return false;
  }

  // Try to enable L2 band.
  if (enableGalileoL2()) {
    std::cout << "Galileo L1/L2 is enabled." << std::endl;
    return true;
  }

  // Try to enable L5 band.
  if (enableGalileoL5()) {
    std::cout << "Galileo L1/L5 is enabled." << std::endl;
    return true;
  }

  std::cerr << "Failed to enable either Galileo L2 or L5 bands." << std::endl;
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
    std::cerr << "Failed to enable BeiDou." << std::endl;
    return false;
  }

  // Enable L1 band.
  if (!enableBeiDouL1()) {
    std::cerr << "Failed to enable BeiDou L1." << std::endl;
    return false;
  }

  // Try to enable L2 band.
  if (enableBeiDouL2()) {
    std::cout << "BeiDou L1/L2 is enabled." << std::endl;
    return true;
  }

  // Try to enable L5 band.
  if (enableBeiDouL5()) {
    std::cout << "BeiDou L1/L5 is enabled." << std::endl;
    return true;
  }

  std::cerr << "Failed to enable either BeiDou L2 or L5 bands." << std::endl;
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
    std::cerr << "Failed to enable QZSS." << std::endl;
    return false;
  }

  // Enable L1 band.
  if (!enableQzssL1()) {
    std::cerr << "Failed to enable QZSS L1." << std::endl;
    return false;
  }

  // Try to enable L2 band.
  if (enableQzssL2()) {
    std::cout << "QZSS L1/L2 is enabled." << std::endl;
    return true;
  }

  // Try to enable L5 band.
  if (enableQzssL5()) {
    std::cout << "QZSS L1/L5 is enabled." << std::endl;
    return true;
  }

  std::cerr << "Failed to enable either QZSS L2 or L5 bands." << std::endl;
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
    std::cerr << "Failed to enable GLONASS." << std::endl;
    return false;
  }

  // Enable L1 band.
  if (!enableGlonassL1()) {
    std::cerr << "Failed to enable GLONASS L1." << std::endl;
    return false;
  }

  // Try to enable L2 band.
  if (enableGlonassL2()) {
    std::cout << "GLONASS L1/L2 is enabled." << std::endl;
    return true;
  }

  std::cout << "GLONASS L1 is enabled." << std::endl;
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
    std::cerr << "Failed to enable NavIC." << std::endl;
    return false;
  }

  // Enable L5 band.
  if (!enableNavIcL5()) {
    std::cerr << "Failed to enable NavIC L5." << std::endl;
    return false;
  }

  std::cout << "NavIC L5 is enabled." << std::endl;
  return false;
}

bool ZEDF9P::disableNavIc()
{
  return enableNavIc(false);
}

bool ZEDF9P::enableSpiProtocol_UBX(bool enable_input, bool enable_output)
{
  return enableSpiInputProtocol(UBX, enable_input) && enableSpiOutputProtocol(UBX, enable_output);
}

bool ZEDF9P::enableSpiProtocol_NMEA(bool enable_input, bool enable_output)
{
  return enableSpiInputProtocol(NMEA, enable_input) && enableSpiOutputProtocol(NMEA, enable_output);
}

bool ZEDF9P::enableSpiProtocol_RTCM3X(bool enable_input, bool enable_output)
{
  return enableSpiInputProtocol(RTCM3X, enable_input) && enableSpiOutputProtocol(RTCM3X, enable_output);
}

bool ZEDF9P::enableSpiProtocol_SPARTN(bool enable_input)
{
  return enableSpiInputProtocol(SPARTN, enable_input);
}

bool ZEDF9P::setAntennaLength(uint8_t length_m)
{
  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_TP, 0x01, length_m * kRG174CableDelay);  // CFG-TP-ANT_CABLEDELAY
}

bool ZEDF9P::enableUsb(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_USB, 0x01, enable);  // CFG-USB-ENABLED
}

bool ZEDF9P::sendMessage(UbxClass cls, uint8_t id, const void* msg, uint16_t size)
{
  UbxHeader header;
  header.sync1 = kUbxSync1;
  header.sync2 = kUbxSync2;
  header.cls = cls;
  header.id = id;
  header.length = size;

  const auto payload_pos = spliceMemory(message_buf_, &header, sizeof(UbxHeader), 0);
  const auto checksum_pos = spliceMemory(message_buf_, msg, size, payload_pos);

  const auto ck = computeChecksum(message_buf_, checksum_pos);
  const auto message_length = spliceMemory(message_buf_, &ck, sizeof(CheckSum), checksum_pos);

  return transport_->send(message_buf_, message_length);
}

bool ZEDF9P::waitForAcknowledge(UbxClass cls, uint8_t id)
{
  payload::ACK_ACK ack;
  payload::ACK_NAK nak;

  const auto cls_str = std::to_string(int(cls));
  const auto id_str = std::to_string(int(id));

  const auto deadline = ch::steady_clock::now() + kWaitForGnssAck;

  while (ch::steady_clock::now() < deadline) {
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
          std::cerr << "An acknowledment message for an unspecified message is received." << std::endl;
          return false;
        }

        break;

      case ACK_NAK:
        nak.decode(payload());

        if (nak.clsID == cls && nak.msgID == id) {
          std::cerr << "Configuration was rejected: (class, id) = (" << cls_str << ", " << id_str << ")" << std::endl;
          return false;
        }
        else {
          std::cerr << "A non-acknowledment message for an unspecified message is received." << std::endl;
          return false;
        }

        break;

      default:
        std::cerr << "Unexpected ACK ID: " << (int)latestId() << std::endl;
        break;
    }
  }

  std::cerr << "Acknowledment message not received: (class, id) = (" << cls_str << ", " << id_str << ")" << std::endl;
  return false;
}

bool ZEDF9P::configure(UbxCfgId cfg_id, const void* msg, uint16_t size)
{
  return sendMessage(CLASS_CFG, cfg_id, msg, size) && waitForAcknowledge(CLASS_CFG, cfg_id);
}

bool ZEDF9P::verifyMessage() const
{
  // Sync chars
  if (*scanner_.getSync1() != kUbxSync1 || *scanner_.getSync2() != kUbxSync2) {
    std::cerr << "The current message is not UBX format." << std::endl;
    return false;
  }

  // Checksum
  uint8_t CK_A = 0, CK_B = 0;
  for (auto x = scanner_.getClass(); x < scanner_.getChecksumA(); ++x) {
    CK_A += *x;
    CK_B += CK_A;
  }
  if (CK_A != *scanner_.getChecksumA() || CK_B != *scanner_.getChecksumB()) {
    std::cerr << "Checksum failed." << std::endl;
    return false;
  }

  return true;
}

bool ZEDF9P::enableGps(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1F, enable);  // CFG-SIGNAL-GPS_ENA
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

bool ZEDF9P::enableSbas(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x20, enable);  // CFG-SIGNAL-SBAS_ENA
}

bool ZEDF9P::enableSbasL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x05, true);  // CFG-SIGNAL-SBAS_L1CA_ENA
}

bool ZEDF9P::enableGalileo(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x21, enable);  // CFG-SIGNAL-GAL_ENA
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

bool ZEDF9P::enableBeiDou(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x22, enable);  // CFG-SIGNAL-BDS_ENA
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

bool ZEDF9P::enableQzss(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x24, enable);  // CFG-SIGNAL-QZSS_ENA
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

bool ZEDF9P::enableGlonass(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x25, enable);  // CFG-SIGNAL-GLO_ENA
}

bool ZEDF9P::enableGlonassL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x18, true);  // CFG-SIGNAL-GLO_L1_ENA
}

bool ZEDF9P::enableGlonassL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1A, true);  // CFG-SIGNAL-GLO_L2_ENA
}

bool ZEDF9P::enableNavIc(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x26, enable);  // CFG-SIGNAL-NAVIC_ENA
}

bool ZEDF9P::enableNavIcL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1D, true);  // CFG-SIGNAL-NAVIC_L5_ENA
}

bool ZEDF9P::enableSpiInputProtocol(CfgProtocol prot, bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SPIINPROT, prot, enable);  // CFG-SPIINPROT-XXX
}

bool ZEDF9P::enableSpiOutputProtocol(CfgProtocol prot, bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SPIOUTPROT, prot, enable);  // CFG-SPIOUTPROT-XXX
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
  std::memmove(dest + dest_offset, src, size);
  return dest_offset + size;
}

uint32_t ZEDF9P::configKeyID(CfgSize size, CfgGroup group, uint8_t id)
{
  return (size << 28) | (group << 16) | id;
}
}  // namespace ublox
}  // namespace tobas
