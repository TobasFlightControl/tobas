// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ublox_gnss.hpp"

#include <cassert>
#include <cstring>
#include <memory>
#include <utility>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas
{
namespace ublox
{
UbloxGnss::UbloxGnss(std::unique_ptr<UbxTransport> _transport) : transport_(std::move(_transport)), scan_rate_(50us)
{
  assert(transport_);
}

bool UbloxGnss::initialize()
{
  return transport_->initialize();
}

void UbloxGnss::setReceiverProfile(ReceiverProfile profile)
{
  receiver_profile_ = profile;
}

std::optional<std::string> UbloxGnss::getModuleName()
{
  if (!sendMessage(CLASS_MON, MON_VER, nullptr, 0)) {
    return std::nullopt;
  }

  const auto deadline = ch::steady_clock::now() + 1s;

  while (ch::steady_clock::now() < deadline) {
    const auto remaining = ch::duration_cast<ch::milliseconds>(deadline - ch::steady_clock::now());

    if (!update(true, remaining)) {
      return std::nullopt;
    }

    if (latestClass() == CLASS_MON && latestId() == MON_VER) {
      break;
    }
  }

  if (latestClass() != CLASS_MON || latestId() != MON_VER) {
    return std::nullopt;
  }

  const auto* length = scanner_.getLength();
  const uint16_t payload_length =
    static_cast<uint16_t>(length[0]) |
    (static_cast<uint16_t>(length[1]) << 8);

  const auto* data = payload();

  constexpr uint16_t kMonVerFixedLength = 40;
  constexpr uint16_t kExtensionLength = 30;

  if (payload_length < kMonVerFixedLength) {
    return std::nullopt;
  }

  for (uint16_t offset = kMonVerFixedLength;
       offset + kExtensionLength <= payload_length;
       offset += kExtensionLength) {

    const char* extension =
      reinterpret_cast<const char*>(&data[offset]);

    constexpr char kModelPrefix[] = "MOD=";

    if (std::strncmp(extension, kModelPrefix, 4) == 0) {
      const char* model = extension + 4;

      const size_t model_length =
        strnlen(model, kExtensionLength - 4);

      return std::string(model, model_length);
    }
  }

  return std::nullopt;
}

bool UbloxGnss::update(bool blocking, std::chrono::milliseconds timeout)
{
  scanner_.reset();

  const bool use_timeout = timeout.count() > 0;
  const auto deadline = ch::steady_clock::now() + timeout;

  if (!blocking) {
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
    if (use_timeout && ch::steady_clock::now() >= deadline) {
      return false;
    }

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

bool UbloxGnss::enableSpiMessage(UbxClass cls, uint8_t id, bool enable)
{
  constexpr char kNotImplemented[] = "Not implemented.";
  constexpr char kNotReceivable[] = "Not receivable.";

  CfgValSet<uint8_t, 1> cfg;

  switch (cls) {
    case CLASS_ACK: {
      std::cerr << kNotReceivable << std::endl;
      return false;
    }
    case CLASS_CFG: {
      std::cerr << kNotReceivable << std::endl;
      return false;
    }
    case CLASS_INF: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_LOG: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_MGA: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_MON: {
      std::cerr << kNotImplemented << std::endl;  // TODO
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
          std::cerr << kNotReceivable << std::endl;
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
          std::cerr << kNotImplemented << std::endl;  // TODO
          return false;
      }
      break;
    }
    case CLASS_NAV2: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_RXM: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_SEC: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_TIM: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_UPD: {
      std::cerr << kNotImplemented << std::endl;  // TODO
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

bool UbloxGnss::configureDynamicsModel(DynamicsModel model)
{
  return cfgValSetSingle<uint8_t>(ONE_BYTE, CFG_NAVSPG, 0x21, model);  // CFG-NAVSPG-DYNMODEL
}

bool UbloxGnss::configureMeasurementRate(uint16_t period_ms)
{
  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_RATE, 0x01, period_ms);  // CFG-RATE-MEAS
}

bool UbloxGnss::configureUartBaudRate(uint32_t baud_rate)
{
  CfgValSet<uint32_t, 1> cfg;
  cfg.data[0].key = configKeyID(FOUR_BYTES, CFG_UART1, 0x01);
  cfg.data[0].value = baud_rate;
  return sendMessage(CLASS_CFG, CFG_VALSET, &cfg, sizeof(cfg));
}

bool UbloxGnss::enableUartMessage(UbxClass cls, uint8_t id, bool enable)
{
  constexpr char kNotImplemented[] = "Not implemented.";
  constexpr char kNotReceivable[] = "Not receivable.";

  CfgValSet<uint8_t, 1> cfg;

  switch (cls) {
    case CLASS_ACK: {
      std::cerr << kNotReceivable << std::endl;
      return false;
    }
    case CLASS_CFG: {
      std::cerr << kNotReceivable << std::endl;
      return false;
    }
    case CLASS_INF: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_LOG: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_MGA: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_MON: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_NAV: {
      switch (id) {
        case NAV_CLOCK:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x66);  // CFG-MSGOUT-UBX_NAV_CLOCK_UART
          break;
        case NAV_COV:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x84);  // CFG-MSGOUT-UBX_NAV_COV_UART
          break;
        case NAV_DOP:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x39);  // CFG-MSGOUT-UBX_NAV_DOP_UART
          break;
        case NAV_EOE:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x60);  // CFG-MSGOUT-UBX_NAV_EOE_UART
          break;
        case NAV_GEOFENCE:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0xA2);  // CFG-MSGOUT-UBX_NAV_GEOFENCE_UART
          break;
        case NAV_HPPOSECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x2F);  // CFG-MSGOUT-UBX_NAV_HPPOSECEF_UART
          break;
        case NAV_HPPOSLLH:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x34);  // CFG-MSGOUT-UBX_NAV_HPPOSLLH_UART
          break;
        case NAV_ODO:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x7F);  // CFG-MSGOUT-UBX_NAV_ODO_UART
          break;
        case NAV_ORB:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x11);  // CFG-MSGOUT-UBX_NAV_ORB_UART
          break;
        case NAV_PL:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x16);  // CFG-MSGOUT-UBX_NAV_PL_UART
          break;
        case NAV_POSECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x25);  // CFG-MSGOUT-UBX_NAV_POSECEF_UART
          break;
        case NAV_POSLLH:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x2A);  // CFG-MSGOUT-UBX_NAV_POSLLH_UART
          break;
        case NAV_PVT:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x07);  // CFG-MSGOUT-UBX_NAV_PVT_UART
          break;
        case NAV_RELPOSNED:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x8E);  // CFG-MSGOUT-UBX_NAV_RELPOSNED_UART
          break;
        case NAV_RESETODO:
          std::cerr << kNotReceivable << std::endl;
          return false;
        case NAV_SAT:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x16);  // CFG-MSGOUT-UBX_NAV_SAT_UART
          break;
        case NAV_SBAS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x6B);  // CFG-MSGOUT-UBX_NAV_SBAS_UART
          break;
        case NAV_SIG:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x46);  // CFG-MSGOUT-UBX_NAV_SIG_UART
          break;
        case NAV_SLAS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x37);  // CFG-MSGOUT-UBX_NAV_SLAS_UART
          break;
        case NAV_STATUS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x1B);  // CFG-MSGOUT-UBX_NAV_STATUS_UART
          break;
        case NAV_SVIN:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x89);  // CFG-MSGOUT-UBX_NAV_SVIN_UART
          break;
        case NAV_TIMEBDS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x52);  // CFG-MSGOUT-UBX_NAV_TIMEBDS_UART
          break;
        case NAV_TIMEGAL:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x57);  // CFG-MSGOUT-UBX_NAV_TIMEGAL_UART
          break;
        case NAV_TIMEGLO:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x4D);  // CFG-MSGOUT-UBX_NAV_TIMEGLO_UART
          break;
        case NAV_TIMEGPS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x48);  // CFG-MSGOUT-UBX_NAV_TIMEGPS_UART
          break;
        case NAV_TIMELS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x61);  // CFG-MSGOUT-UBX_NAV_TIMELS_UART
          break;
        case NAV_TIMEQZSS:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x87);  // CFG-MSGOUT-UBX_NAV_TIMEQZSS_UART
          break;
        case NAV_TIMEUTC:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x5C);  // CFG-MSGOUT-UBX_NAV_TIMEUTC_UART
          break;
        case NAV_VELECEF:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x3E);  // CFG-MSGOUT-UBX_NAV_VELECEF_UART
          break;
        case NAV_VELNED:
          cfg.data[0].key = configKeyID(ONE_BYTE, CFG_MSGOUT, 0x43);  // CFG-MSGOUT-UBX_NAV_VELNED_UART
          break;
        default:
          std::cerr << kNotImplemented << std::endl;  // TODO
          return false;
      }
      break;
    }
    case CLASS_NAV2: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_RXM: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_SEC: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_TIM: {
      std::cerr << kNotImplemented << std::endl;  // TODO
      return false;
    }
    case CLASS_UPD: {
      std::cerr << kNotImplemented << std::endl;  // TODO
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

bool UbloxGnss::enableGps()
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

  // Enable L2 band for ZED-F9P.
  if(receiver_profile_ == F9P) {
    if (!enableGpsL2()) {
      std::cerr << "Failed to enable GPS L2." << std::endl;
      return false;
    }

    std::cout << "GPS L1/L2 is enabled." << std::endl;
    return true;

  }    // Enable L2 and L5 band for ZED-X20P.
  if (receiver_profile_ == X20P) {
    if (!enableGpsL2()) {
      std::cerr << "Failed to enable GPS L2." << std::endl;
      return false;
    }

    if (!enableGpsL5()) {
      std::cerr << "Failed to enable GPS L5." << std::endl;
      return false;
    }

    std::cout << "GPS L1/L2/L5 is enabled." << std::endl;
    return true;
  }

  return false;
}

bool UbloxGnss::disableGps()
{
  return enableGps(false);
}

bool UbloxGnss::enableSbas()
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

bool UbloxGnss::disableSbas()
{
  return enableGps(false);
}

bool UbloxGnss::enableGalileo()
{
  // Enable Galileo.
  if (!enableGalileo(true)) {
    std::cerr << "Failed to enable Galileo." << std::endl;
    return false;
  }

  // Enable E1 band.
  if (!enableGalileoE1()) {
    std::cerr << "Failed to enable Galileo E1." << std::endl;
    return false;
  }

  /// Enable E5b band for ZED-F9P.
  if (receiver_profile_ == F9P) {
    if (!enableGalileoE5b()) {
      std::cerr << "Failed to enable Galileo E5b." << std::endl;
      return false;
    }

    std::cout << "Galileo E1/E5b is enabled." << std::endl;
    return true;
  }

  // Enable E5a and E6 bands for ZED-X20P.
  if (receiver_profile_ == X20P) {
    if (!enableGalileoE5a()) {
      std::cerr << "Failed to enable Galileo E5a." << std::endl;
      return false;
    }

    if (!enableGalileoE6()) {
      std::cerr << "Failed to enable Galileo E6." << std::endl;
      return false;
    }

    std::cout << "Galileo E1/E5a/E6 is enabled." << std::endl;
    return true;
  }

  return false;
}

bool UbloxGnss::disableGalileo()
{
  return enableGalileo(false);
}

bool UbloxGnss::enableBeiDou()
{
  // Enable BeiDou.
  if (!enableBeiDou(true)) {
    std::cerr << "Failed to enable BeiDou." << std::endl;
    return false;
  }

  // Enable B1I bands
  if (!enableBeiDouB1I()) {
    std::cerr << "Failed to enable GPS B1I." << std::endl;
    return false;
  }

  // Enable B2I bands for ZED-F9P.
  if (receiver_profile_ == F9P) {
    if (!enableBeiDouB2I()) {
      std::cerr << "Failed to enable BeiDou B2I." << std::endl;
      return false;
    }

    std::cout << "BeiDou B1I/B2I is enabled." << std::endl;
    return true;
  }

  // Enable B1C, B2a and B3I bands for ZED-X20P.
  if (receiver_profile_ == X20P) {
    if (!enableBeiDouB1C()) {
      std::cerr << "Failed to enable BeiDou B1C." << std::endl;
      return false;
    }

    if (!enableBeiDouB2a()) {
      std::cerr << "Failed to enable BeiDou B2a." << std::endl;
      return false;
    }

    if (!enableBeiDouB3I()) {
      std::cerr << "Failed to enable BeiDou B3I." << std::endl;
      return false;
    }

    std::cout << "BeiDou B1I/B1C/B2a/B3I is enabled." << std::endl;
    return true;
  }

  return false;
}

bool UbloxGnss::disableBeiDou()
{
  return enableBeiDou(false);
}

bool UbloxGnss::enableQzss()
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

  // Enable L2 band for ZED-F9P.
  if(receiver_profile_ == F9P) {
    if (!enableQzssL2()) {
      std::cerr << "Failed to enable QZSS L2." << std::endl;
      return false;
    }

    std::cout << "QZSS L1/L2 is enabled." << std::endl;
    return true;

  }
  // Enable L2 and L5 band for ZED-X20P.
  if (receiver_profile_ == X20P) {
    if (!enableQzssL2()) {
      std::cerr << "Failed to enable QZSS L2." << std::endl;
      return false;
    }

    if (!enableQzssL5()) {
      std::cerr << "Failed to enable QZSS L5." << std::endl;
      return false;
    }

    std::cout << "QZSS L1/L2/L5 is enabled." << std::endl;
    return true;
  }

  return false;
}

bool UbloxGnss::disableQzss()
{
  return enableQzss(false);
}

bool UbloxGnss::enableGlonass()
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

bool UbloxGnss::disableGlonass()
{
  return enableGlonass(false);
}

bool UbloxGnss::enableNavIc()
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
  return true;
}

bool UbloxGnss::disableNavIc()
{
  return enableNavIc(false);
}

bool UbloxGnss::enableSpiProtocol_UBX(bool enable_input, bool enable_output)
{
  return enableSpiInputProtocol(UBX, enable_input) && enableSpiOutputProtocol(UBX, enable_output);
}

bool UbloxGnss::enableSpiProtocol_NMEA(bool enable_input, bool enable_output)
{
  return enableSpiInputProtocol(NMEA, enable_input) && enableSpiOutputProtocol(NMEA, enable_output);
}

bool UbloxGnss::enableSpiProtocol_RTCM3X(bool enable_input, bool enable_output)
{
  return enableSpiInputProtocol(RTCM3X, enable_input) && enableSpiOutputProtocol(RTCM3X, enable_output);
}

bool UbloxGnss::enableSpiProtocol_SPARTN(bool enable_input)
{
  return enableSpiInputProtocol(SPARTN, enable_input);
}

bool UbloxGnss::enableUartProtocol_UBX(bool enable_input, bool enable_output)
{
  return enableUartInputProtocol(UBX, enable_input) && enableUartOutputProtocol(UBX, enable_output);
}

bool UbloxGnss::enableUartProtocol_NMEA(bool enable_input, bool enable_output)
{
  return enableUartInputProtocol(NMEA, enable_input) && enableUartOutputProtocol(NMEA, enable_output);
}

bool UbloxGnss::enableUartProtocol_RTCM3X(bool enable_input, bool enable_output)
{
  return enableUartInputProtocol(RTCM3X, enable_input) && enableUartOutputProtocol(RTCM3X, enable_output);
}

bool UbloxGnss::enableUartProtocol_SPARTN(bool enable_input)
{
  return enableUartInputProtocol(SPARTN, enable_input);
}
bool UbloxGnss::setAntennaLength(uint8_t length_m)
{
  constexpr uint8_t kRG174CableDelay = 5;  // [ns/m] Coaxial cable delay.
  return cfgValSetSingle<uint16_t>(TWO_BYTES, CFG_TP, 0x01, length_m * kRG174CableDelay);  // CFG-TP-ANT_CABLEDELAY
}

bool UbloxGnss::enableUsb(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_USB, 0x01, enable);  // CFG-USB-ENABLED
}

bool UbloxGnss::sendMessage(UbxClass cls, uint8_t id, const void* msg, uint16_t size)
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

  return transport_->send(tx_buf_, message_length);
}

bool UbloxGnss::waitForAcknowledge(UbxClass cls, uint8_t id)
{
  payload::ACK_ACK ack;
  payload::ACK_NAK nak;

  const auto cls_str = std::to_string(static_cast<int>(cls));
  const auto id_str = std::to_string(static_cast<int>(id));

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
        std::cerr << "Unexpected ACK ID: " << static_cast<int>(latestId()) << std::endl;
        break;
    }
  }

  std::cerr << "Acknowledment message not received: (class, id) = (" << cls_str << ", " << id_str << ")" << std::endl;
  return false;
}

bool UbloxGnss::configure(UbxCfgId cfg_id, const void* msg, uint16_t size)
{
  return sendMessage(CLASS_CFG, cfg_id, msg, size) && waitForAcknowledge(CLASS_CFG, cfg_id);
}

bool UbloxGnss::verifyMessage() const
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

bool UbloxGnss::enableGps(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1F, enable);  // CFG-SIGNAL-GPS_ENA
}

bool UbloxGnss::enableGpsL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x01, true);  // CFG-SIGNAL-GPS_L1CA_ENA
}

bool UbloxGnss::enableGpsL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x03, true);  // CFG-SIGNAL-GPS_L2C_ENA
}

bool UbloxGnss::enableGpsL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x04, true);  // CFG-SIGNAL-GPS_L5_ENA
}

bool UbloxGnss::enableSbas(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x20, enable);  // CFG-SIGNAL-SBAS_ENA
}

bool UbloxGnss::enableSbasL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x05, true);  // CFG-SIGNAL-SBAS_L1CA_ENA
}

bool UbloxGnss::enableGalileo(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x21, enable);  // CFG-SIGNAL-GAL_ENA
}

bool UbloxGnss::enableGalileoE1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x07, true);  // CFG-SIGNAL-GAL_E1_ENA
}

bool UbloxGnss::enableGalileoE5b()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0A, true);  // CFG-SIGNAL-GAL_E5B_ENA
}

bool UbloxGnss::enableGalileoE5a()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x09, true);  // CFG-SIGNAL-GAL_E5A_ENA
}

bool UbloxGnss::enableGalileoE6()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0B, true);  // CFG-SIGNAL-GAL_E6_ENA
}

bool UbloxGnss::enableBeiDou(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x22, enable);  // CFG-SIGNAL-BDS_ENA
}

bool UbloxGnss::enableBeiDouB1I()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0D, true);  // CFG-SIGNAL-BDS_B1_ENA
}

bool UbloxGnss::enableBeiDouB1C()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0F, true);  // CFG-SIGNAL-BDS_B1C_ENA
}

bool UbloxGnss::enableBeiDouB2I()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x0E, true);  // CFG-SIGNAL-BDS_B2I_ENA
}

bool UbloxGnss::enableBeiDouB3I()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x10, true);  // CFG-SIGNAL-BDS_B3_ENA
}

bool UbloxGnss::enableBeiDouB2a()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x28, true);  // CFG-SIGNAL-BDS_B2A_ENA
}

bool UbloxGnss::enableQzss(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x24, enable);  // CFG-SIGNAL-QZSS_ENA
}

bool UbloxGnss::enableQzssL1()
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

bool UbloxGnss::enableQzssL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x15, true);  // CFG-SIGNAL-QZSS_L2C_ENA
}

bool UbloxGnss::enableQzssL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x17, true);  // CFG-SIGNAL-QZSS_L5_ENA
}

bool UbloxGnss::enableGlonass(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x25, enable);  // CFG-SIGNAL-GLO_ENA
}

bool UbloxGnss::enableGlonassL1()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x18, true);  // CFG-SIGNAL-GLO_L1_ENA
}

bool UbloxGnss::enableGlonassL2()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1A, true);  // CFG-SIGNAL-GLO_L2_ENA
}

bool UbloxGnss::enableNavIc(bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x26, enable);  // CFG-SIGNAL-NAVIC_ENA
}

bool UbloxGnss::enableNavIcL5()
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SIGNAL, 0x1D, true);  // CFG-SIGNAL-NAVIC_L5_ENA
}

bool UbloxGnss::enableSpiInputProtocol(CfgProtocol prot, bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SPIINPROT, prot, enable);  // CFG-SPIINPROT-XXX
}

bool UbloxGnss::enableSpiOutputProtocol(CfgProtocol prot, bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_SPIOUTPROT, prot, enable);  // CFG-SPIOUTPROT-XXX
}

bool UbloxGnss::enableUartInputProtocol(CfgProtocol prot, bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_UART1INPROT, prot, enable);  // CFG-UART1INPROT-XXX
}

bool UbloxGnss::enableUartOutputProtocol(CfgProtocol prot, bool enable)
{
  return cfgValSetSingle<uint8_t>(ONE_BIT, CFG_UART1OUTPROT, prot, enable);  // CFG-UART1OUTPROT-XXX
}

UbloxGnss::CheckSum UbloxGnss::computeChecksum(const uint8_t* message, size_t checksum_pos)
{
  CheckSum ck;
  ck.CK_A = ck.CK_B = 0;

  for (size_t i = kUbxSyncLength; i < checksum_pos; ++i) {
    ck.CK_A += message[i];
    ck.CK_B += ck.CK_A;
  }

  return ck;
}

size_t UbloxGnss::spliceMemory(uint8_t* dest, const void* src, size_t size, size_t dest_offset)
{
  std::memmove(dest + dest_offset, src, size);
  return dest_offset + size;
}

uint32_t UbloxGnss::configKeyID(CfgSize size, CfgGroup group, uint8_t id)
{
  return (size << 28) | (group << 16) | id;
}
}  // namespace ublox
}  // namespace tobas
