// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/gnss_receiver.hpp"

#include <cassert>
#include <cstring>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <tobas_time_tools/rate.hpp>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas::ublox
{
GnssReceiver::GnssReceiver(std::unique_ptr<UbxTransport> _transport) : transport_(std::move(_transport))
{
  assert(transport_);
}

bool GnssReceiver::initialize()
{
  receiver_profile_.reset();
  scanner_.reset();
  if (!transport_->initialize()) {
    return false;
  }

  const auto receiver_profile = detectReceiverProfile();
  if (!receiver_profile) {
    std::cerr << receiver_profile.error() << '\n';
    return false;
  }

  if (*receiver_profile == F9P) {
    if (!disableGlonass()) {
      return false;
    }
  }
  else if (*receiver_profile == X20P) {
    if (!enableNavIc()) {
      return false;
    }
  }

  if (!configureMeasurementRate(*receiver_profile == F9P ? 50 : 40)) {
    return false;
  }
  receiver_profile_ = *receiver_profile;
  return true;
}

std::expected<GnssReceiver::ReceiverProfile, std::string> GnssReceiver::detectReceiverProfile()
{
  const auto module = transport_->pollVersion(1s);
  if (!module) {
    return std::unexpected(module.error());
  }
  if (*module == "ZED-F9P") {
    return F9P;
  }
  if (*module == "ZED-X20P") {
    return X20P;
  }
  return std::unexpected("Unsupported GNSS module: " + *module);
}

bool GnssReceiver::update(bool _blocking)
{
  return transport_->receiveMessage(scanner_, _blocking, _blocking ? 1s : 1ms);
}

bool GnssReceiver::enableMessage(UbxClass _cls, uint8_t _id, bool _enable)
{
  constexpr char kNotImplemented[] = "Not implemented.\n";
  constexpr char kNotReceivable[] = "Not receivable.\n";

  CfgValSet<uint8_t, 1> cfg;
  uint16_t item_id = 0;

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
      switch (_id) {
        case NAV_CLOCK:
          item_id = 0x069;  // CFG-MSGOUT-UBX_NAV_CLOCK_SPI
          break;
        case NAV_COV:
          item_id = 0x087;  // CFG-MSGOUT-UBX_NAV_COV_SPI
          break;
        case NAV_DOP:
          item_id = 0x03C;  // CFG-MSGOUT-UBX_NAV_DOP_SPI
          break;
        case NAV_EOE:
          item_id = 0x163;  // CFG-MSGOUT-UBX_NAV_EOE_SPI
          break;
        case NAV_GEOFENCE:
          item_id = 0x0A5;  // CFG-MSGOUT-UBX_NAV_GEOFENCE_SPI
          break;
        case NAV_HPPOSECEF:
          item_id = 0x032;  // CFG-MSGOUT-UBX_NAV_HPPOSECEF_SPI
          break;
        case NAV_HPPOSLLH:
          item_id = 0x037;  // CFG-MSGOUT-UBX_NAV_HPPOSLLH_SPI
          break;
        case NAV_ODO:
          item_id = 0x082;  // CFG-MSGOUT-UBX_NAV_ODO_SPI
          break;
        case NAV_ORB:
          item_id = 0x014;  // CFG-MSGOUT-UBX_NAV_ORB_SPI
          break;
        case NAV_PL:
          item_id = 0x419;  // CFG-MSGOUT-UBX_NAV_PL_SPI
          break;
        case NAV_POSECEF:
          item_id = 0x028;  // CFG-MSGOUT-UBX_NAV_POSECEF_SPI
          break;
        case NAV_POSLLH:
          item_id = 0x02D;  // CFG-MSGOUT-UBX_NAV_POSLLH_SPI
          break;
        case NAV_PVT:
          item_id = 0x00A;  // CFG-MSGOUT-UBX_NAV_PVT_SPI
          break;
        case NAV_RELPOSNED:
          item_id = 0x091;  // CFG-MSGOUT-UBX_NAV_RELPOSNED_SPI
          break;
        case NAV_RESETODO:
          std::cerr << kNotReceivable;
          return false;
        case NAV_SAT:
          item_id = 0x019;  // CFG-MSGOUT-UBX_NAV_SAT_SPI
          break;
        case NAV_SBAS:
          item_id = 0x06E;  // CFG-MSGOUT-UBX_NAV_SBAS_SPI
          break;
        case NAV_SIG:
          item_id = 0x349;  // CFG-MSGOUT-UBX_NAV_SIG_SPI
          break;
        case NAV_SLAS:
          item_id = 0x33A;  // CFG-MSGOUT-UBX_NAV_SLAS_SPI
          break;
        case NAV_STATUS:
          item_id = 0x01E;  // CFG-MSGOUT-UBX_NAV_STATUS_SPI
          break;
        case NAV_SVIN:
          item_id = 0x08C;  // CFG-MSGOUT-UBX_NAV_SVIN_SPI
          break;
        case NAV_TIMEBDS:
          item_id = 0x055;  // CFG-MSGOUT-UBX_NAV_TIMEBDS_SPI
          break;
        case NAV_TIMEGAL:
          item_id = 0x05A;  // CFG-MSGOUT-UBX_NAV_TIMEGAL_SPI
          break;
        case NAV_TIMEGLO:
          item_id = 0x050;  // CFG-MSGOUT-UBX_NAV_TIMEGLO_SPI
          break;
        case NAV_TIMEGPS:
          item_id = 0x04B;  // CFG-MSGOUT-UBX_NAV_TIMEGPS_SPI
          break;
        case NAV_TIMELS:
          item_id = 0x064;  // CFG-MSGOUT-UBX_NAV_TIMELS_SPI
          break;
        case NAV_TIMEQZSS:
          item_id = 0x38A;  // CFG-MSGOUT-UBX_NAV_TIMEQZSS_SPI
          break;
        case NAV_TIMEUTC:
          item_id = 0x05F;  // CFG-MSGOUT-UBX_NAV_TIMEUTC_SPI
          break;
        case NAV_VELECEF:
          item_id = 0x041;  // CFG-MSGOUT-UBX_NAV_VELECEF_SPI
          break;
        case NAV_VELNED:
          item_id = 0x046;  // CFG-MSGOUT-UBX_NAV_VELNED_SPI
          break;
        default:
          std::cerr << kNotImplemented;  // TODO
          return false;
      }
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

  switch (transport_->interface()) {
    case UbxTransport::Interface::SPI:
      break;
    case UbxTransport::Interface::UART: {
      // For these NAV messages, UART1 item IDs are three below the SPI item IDs.
      constexpr uint16_t kUart1ItemOffset = 3;
      item_id -= kUart1ItemOffset;
      break;
    }
    default:
      std::cerr << kNotImplemented;
      return false;
  }

  cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, item_id);
  cfg.data[0].value = _enable ? 1 : 0;  // Maximum rate if enabled.

  return configure(CFG_VALSET, &cfg, sizeof(cfg));
}

bool GnssReceiver::configureDynamicsModel(DynamicsModel _model)
{
  return cfgValSetSingle<uint8_t>(ONE_BYTE, CFG_NAVSPG, 0x21, _model);  // CFG-NAVSPG-DYNMODEL
}

bool GnssReceiver::configureMeasurementRate(uint16_t _period_ms)
{
  if (!cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_RATE, 0x01, _period_ms)) {  // CFG-RATE-MEAS
    return false;
  }

  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_RATE, 0x02, 1);  // CFG-RATE-NAV
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

  // Enable L2 band for ZED-F9P.
  if (receiver_profile_ == F9P) {
    if (!enableGpsL2()) {
      std::cerr << "Failed to enable GPS L2.\n";
      return false;
    }

    std::cout << "GPS L1/L2 is enabled.\n";
    return true;
  }

  // Enable L2 and L5 bands for ZED-X20P.
  if (receiver_profile_ == X20P) {
    if (!enableGpsL2()) {
      std::cerr << "Failed to enable GPS L2.\n";
      return false;
    }

    if (!enableGpsL5()) {
      std::cerr << "Failed to enable GPS L5.\n";
      return false;
    }

    std::cout << "GPS L1/L2/L5 is enabled.\n";
    return true;
  }

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

  // Enable E1 band.
  if (!enableGalileoE1()) {
    std::cerr << "Failed to enable Galileo E1.\n";
    return false;
  }

  // Enable E5b band for ZED-F9P.
  if (receiver_profile_ == F9P) {
    if (!enableGalileoE5b()) {
      std::cerr << "Failed to enable Galileo E5b.\n";
      return false;
    }

    std::cout << "Galileo E1/E5b is enabled.\n";
    return true;
  }

  // Enable E5a and E6 bands for ZED-X20P.
  if (receiver_profile_ == X20P) {
    if (!enableGalileoE5a()) {
      std::cerr << "Failed to enable Galileo E5a.\n";
      return false;
    }

    if (!enableGalileoE6()) {
      std::cerr << "Failed to enable Galileo E6.\n";
      return false;
    }

    std::cout << "Galileo E1/E5a/E6 is enabled.\n";
    return true;
  }

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

  // Enable B1I band.
  if (!enableBeiDouB1I()) {
    std::cerr << "Failed to enable BeiDou B1I.\n";
    return false;
  }

  // Enable B2I bands for ZED-F9P.
  if (receiver_profile_ == F9P) {
    if (!enableBeiDouB2I()) {
      std::cerr << "Failed to enable BeiDou B2I.\n";
      return false;
    }

    std::cout << "BeiDou B1I/B2I is enabled.\n";
    return true;
  }

  // Enable B1C, B2a and B3I bands for ZED-X20P.
  if (receiver_profile_ == X20P) {
    if (!enableBeiDouB1C()) {
      std::cerr << "Failed to enable BeiDou B1C.\n";
      return false;
    }

    if (!enableBeiDouB2a()) {
      std::cerr << "Failed to enable BeiDou B2a.\n";
      return false;
    }

    if (!enableBeiDouB3I()) {
      std::cerr << "Failed to enable BeiDou B3I.\n";
      return false;
    }

    std::cout << "BeiDou B1I/B1C/B2a/B3I is enabled.\n";
    return true;
  }

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

  // Enable L2 band for ZED-F9P.
  if (receiver_profile_ == F9P) {
    if (!enableQzssL2()) {
      std::cerr << "Failed to enable QZSS L2.\n";
      return false;
    }

    std::cout << "QZSS L1/L2 is enabled.\n";
    return true;
  }
  // Enable L2 and L5 band for ZED-X20P.
  if (receiver_profile_ == X20P) {
    if (!enableQzssL2()) {
      std::cerr << "Failed to enable QZSS L2.\n";
      return false;
    }

    if (!enableQzssL5()) {
      std::cerr << "Failed to enable QZSS L5.\n";
      return false;
    }

    std::cout << "QZSS L1/L2/L5 is enabled.\n";
    return true;
  }

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
  return true;
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

bool UbxTransport::sendMessage(uint8_t _cls, uint8_t _id, const void* _msg, uint16_t _size) noexcept
try {
  // Allocate space for the header, payload, and checksum of this message.
  std::vector<uint8_t> tx_buf(sizeof(UbxHeader) + _size + sizeof(CheckSum));

  UbxHeader header;
  header.sync1 = kUbxSync1;
  header.sync2 = kUbxSync2;
  header.cls = _cls;
  header.id = _id;
  header.length = _size;

  const auto payload_pos = spliceMemory(tx_buf.data(), &header, sizeof(UbxHeader), 0);
  auto checksum_pos = payload_pos;
  if (_size != 0) {
    checksum_pos = spliceMemory(tx_buf.data(), _msg, _size, payload_pos);
  }

  const auto ck = computeChecksum(tx_buf.data(), checksum_pos);
  const auto message_length = spliceMemory(tx_buf.data(), &ck, sizeof(CheckSum), checksum_pos);

  return writeBytes(tx_buf.data(), message_length);
}

catch (...) {
  return false;
}

bool UbxTransport::receiveMessage(UbxScanner& _scanner, bool _blocking, ch::milliseconds _timeout) noexcept
try {
  if (_scanner.state() == UbxScanner::kDone) {
    _scanner.reset();
  }
  const auto deadline = ch::steady_clock::now() + _timeout;
  tim::Rate scan_rate(50us);
  while (ch::steady_clock::now() < deadline) {
    const auto byte = receiveByte();
    if (!byte) {
      if (!_blocking) {
        return false;
      }
      std::this_thread::sleep_for(1ms);
      continue;
    }
    _scanner.update(*byte);
    if (interface() == Interface::SPI) {
      scan_rate.sleep();
    }
    if (_scanner.state() == UbxScanner::kDone) {
      const auto checksum = computeChecksum(_scanner.getSync1(), _scanner.messageLength() - kUbxChecksumLength);
      if (checksum.CK_A == *_scanner.getChecksumA() && checksum.CK_B == *_scanner.getChecksumB()) {
        return true;
      }
      _scanner.reset();
    }
  }
  // Retain partial frames for the next receive call.
  return false;
}

catch (...) {
  _scanner.reset();
  return false;
}

UbxTransport::CheckSum UbxTransport::computeChecksum(const uint8_t* _message, size_t _checksum_pos)
{
  CheckSum ck;
  ck.CK_A = ck.CK_B = 0;

  for (size_t i = kUbxSyncLength; i < _checksum_pos; ++i) {
    ck.CK_A += _message[i];
    ck.CK_B += ck.CK_A;
  }

  return ck;
}

size_t UbxTransport::spliceMemory(uint8_t* _dest, const void* _src, size_t _size, size_t _dest_offset)
{
  std::memmove(_dest + _dest_offset, _src, _size);
  return _dest_offset + _size;
}

std::expected<std::string, std::string> UbxTransport::pollVersion(ch::milliseconds _timeout)
{
  UbxScanner scanner;
  // Discard already queued frames and partial bytes before polling at a new speed.
  const auto flush_deadline = ch::steady_clock::now() + 20ms;
  while (ch::steady_clock::now() < flush_deadline) {
    if (!receiveByte()) {
      break;
    }
  }
  if (!sendMessage(0x0A, 0x04, nullptr, 0)) {
    return std::unexpected("Failed to send MON-VER request");
  }
  const auto deadline = ch::steady_clock::now() + _timeout;
  while (ch::steady_clock::now() < deadline) {
    const auto remaining = ch::ceil<ch::milliseconds>(deadline - ch::steady_clock::now());
    if (!receiveMessage(scanner, true, remaining)) {
      break;
    }
    if (*scanner.getClass() != 0x0A || *scanner.getId() != 0x04) {
      continue;
    }
    const auto* length = scanner.getLength();
    payload::MON_VER version(length[0] | (uint16_t{ length[1] } << 8));
    version.decode(scanner.getPayload());
    if (!version.error.empty()) {
      return std::unexpected(version.error);
    }
    return version.module;
  }
  return std::unexpected("MON-VER response timed out");
}

bool GnssReceiver::waitForAcknowledge(UbxClass _cls, uint8_t _id)
{
  const auto deadline = ch::steady_clock::now() + 1s;
  while (ch::steady_clock::now() < deadline) {
    const auto remaining = ch::ceil<ch::milliseconds>(deadline - ch::steady_clock::now());
    if (!transport_->receiveMessage(scanner_, true, remaining)) {
      break;
    }
    if (latestClass() != CLASS_ACK || (latestId() != ACK_ACK && latestId() != ACK_NAK)) {
      continue;
    }
    const auto* length = scanner_.getLength();
    if (length[0] != 2 || length[1] != 0) {
      continue;
    }
    if (payload()[0] != _cls || payload()[1] != _id) {
      continue;
    }
    if (latestId() == ACK_NAK) {
      std::cerr << "Configuration was rejected: (class, id) = (" << static_cast<int>(_cls) << ", "
                << static_cast<int>(_id) << ")\n";
      return false;
    }
    return true;
  }
  std::cerr << "Acknowledgment message not received: (class, id) = (" << static_cast<int>(_cls) << ", "
            << static_cast<int>(_id) << ")\n";
  return false;
}

bool GnssReceiver::configure(UbxCfgId _cfg_id, const void* _msg, uint16_t _size)
{
  const auto deadline = ch::steady_clock::now() + 100ms;
  while (ch::steady_clock::now() < deadline) {
    const auto remaining = ch::ceil<ch::milliseconds>(deadline - ch::steady_clock::now());
    if (!transport_->receiveMessage(scanner_, false, remaining)) {
      break;
    }
  }
  scanner_.reset();
  return transport_->sendMessage(CLASS_CFG, _cfg_id, _msg, _size) && waitForAcknowledge(CLASS_CFG, _cfg_id);
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

bool GnssReceiver::enableGalileoE1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x07, true);  // CFG-SIGNAL-GAL_E1_ENA
}

bool GnssReceiver::enableGalileoE5b()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0A, true);  // CFG-SIGNAL-GAL_E5B_ENA
}

bool GnssReceiver::enableGalileoE5a()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x09, true);  // CFG-SIGNAL-GAL_E5A_ENA
}

bool GnssReceiver::enableGalileoE6()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0B, true);  // CFG-SIGNAL-GAL_E6_ENA
}

bool GnssReceiver::enableBeiDou(bool _enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x22, _enable);  // CFG-SIGNAL-BDS_ENA
}

bool GnssReceiver::enableBeiDouB1I()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0D, true);  // CFG-SIGNAL-BDS_B1_ENA
}

bool GnssReceiver::enableBeiDouB1C()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0F, true);  // CFG-SIGNAL-BDS_B1C_ENA
}

bool GnssReceiver::enableBeiDouB2I()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0E, true);  // CFG-SIGNAL-BDS_B2I_ENA
}

bool GnssReceiver::enableBeiDouB3I()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x10, true);  // CFG-SIGNAL-BDS_B3_ENA
}

bool GnssReceiver::enableBeiDouB2a()
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
  return cfgValSetSingle<uint8_t>(
    ONE_BIT,
    transport_->interface() == UbxTransport::Interface::SPI ? CFG_SPIINPROT : CFG_UART1INPROT,
    _prot,
    _enable);  // Input protocol on the connected interface.
}

bool GnssReceiver::enableOutputProtocol(CfgProtocol _prot, bool _enable)
{
  return cfgValSetSingle<uint8_t>(
    ONE_BIT,
    transport_->interface() == UbxTransport::Interface::SPI ? CFG_SPIOUTPROT : CFG_UART1OUTPROT,
    _prot,
    _enable);  // Output protocol on the connected interface.
}

uint32_t GnssReceiver::configKeyID(CfgSize _size, CfgGroup _group, uint16_t _id)
{
  return (_size << 28) | (_group << 16) | (_id & 0x0FFF);
}
}  // namespace tobas::ublox
