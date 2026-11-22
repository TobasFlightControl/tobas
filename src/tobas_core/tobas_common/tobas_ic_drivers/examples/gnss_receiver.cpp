// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string_view>
#include <utility>

#include <tobas_ic_drivers/ublox/gnss_receiver.hpp>
#include <tobas_ic_drivers/ublox/ubx_transport_spi.hpp>
#include <tobas_ic_drivers/ublox/ubx_transport_uart.hpp>

int main(int _argc, char* _argv[])
{
  if (_argc != 3) {
    const auto filename = _argv[0];
    std::cerr << "Usage: " << filename << " <Interface> <Device>\n";
    return EXIT_FAILURE;
  }

  std::unique_ptr<tobas::ublox::UbxTransport> transport;

  const std::string_view interface(_argv[1]);
  const auto device = _argv[2];
  if (interface == "spi") {
    transport = std::make_unique<tobas::ublox::UbxTransportSpi>(device);
  }
  else if (interface == "uart") {
    transport = std::make_unique<tobas::ublox::UbxTransportUart>(device);
  }
  else {
    std::cerr << "Unsupported interface: " << interface << " (expected 'spi' or 'uart')\n";
    return EXIT_FAILURE;
  }

  tobas::ublox::GnssReceiver gnss(std::move(transport));

  tobas::ublox::payload::NAV_COV cov;
  tobas::ublox::payload::NAV_HPPOSLLH hpposllh;
  tobas::ublox::payload::NAV_POSLLH posllh;
  tobas::ublox::payload::NAV_PVT pvt;
  tobas::ublox::payload::NAV_SAT sat;
  tobas::ublox::payload::NAV_STATUS status;
  tobas::ublox::payload::NAV_TIMEGPS timegps;
  tobas::ublox::payload::NAV_VELNED velned;

  std::cout << "Initializing GNSS device.\n";
  if (!gnss.initialize()) {
    std::cerr << "Failed to initialize GNSS driver.\n";
    return EXIT_FAILURE;
  }

  // Enable GNSS.
  std::cout << "Enabling GNSS.\n";
  if (!gnss.enableGps()) {
    std::cerr << "Failed to enable GPS.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSbas()) {
    std::cerr << "Failed to enable SBAS.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.disableGalileo()) {
    std::cerr << "Failed to disable Galileo.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.disableBeiDou()) {
    std::cerr << "Failed to disable BeiDou.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableQzss()) {
    std::cerr << "Failed to enable QZSS.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.disableGlonass()) {
    std::cerr << "Failed to disable GLONASS.\n";
    return EXIT_FAILURE;
  }

  // Enable messages.
  std::cout << "Enabling messages.\n";
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_COV, true)) {
    std::cerr << "Failed to enable NAV_COV message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_HPPOSLLH, true)) {
    std::cerr << "Failed to enable NAV_HPPOSLLH message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_POSLLH, true)) {
    std::cerr << "Failed to enable NAV_POSLLH message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_PVT, true)) {
    std::cerr << "Failed to enable NAV_PVT message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_SAT, true)) {
    std::cerr << "Failed to enable NAV_SAT message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_STATUS, true)) {
    std::cerr << "Failed to enable NAV_STATUS message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_TIMEGPS, true)) {
    std::cerr << "Failed to enable NAV_TIMEGPS message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableMessage(tobas::ublox::GnssReceiver::CLASS_NAV, tobas::ublox::GnssReceiver::NAV_VELNED, true)) {
    std::cerr << "Failed to enable NAV_VELNED message.\n";
    return EXIT_FAILURE;
  }

  std::cout << "Initial configuration finished successfully.\n";

  while (true) {
    if (!gnss.update(true)) {
      std::cerr << "Failed to receive a GNSS message.\n";
      return EXIT_FAILURE;
    }

    if (gnss.latestClass() != tobas::ublox::GnssReceiver::CLASS_NAV) {
      continue;
    }

    switch (gnss.latestId()) {
      case tobas::ublox::GnssReceiver::NAV_COV:
        cov.decode(gnss.payload());
        std::cout << "[NAV_COV]\n";
        std::cout << cov << '\n';
        break;
      case tobas::ublox::GnssReceiver::NAV_HPPOSLLH:
        hpposllh.decode(gnss.payload());
        std::cout << "[NAV_HPPOSLLH]\n";
        std::cout << hpposllh << '\n';
        break;
      case tobas::ublox::GnssReceiver::NAV_POSLLH:
        posllh.decode(gnss.payload());
        std::cout << "[NAV_POSLLH]\n";
        std::cout << posllh << '\n';
        break;
      case tobas::ublox::GnssReceiver::NAV_PVT:
        pvt.decode(gnss.payload());
        std::cout << "[NAV_PVT]\n";
        std::cout << pvt << '\n';
        break;
      case tobas::ublox::GnssReceiver::NAV_SAT:
        sat.decode(gnss.payload());
        std::cout << "[NAV_SAT]\n";
        std::cout << sat << '\n';
        break;
      case tobas::ublox::GnssReceiver::NAV_STATUS:
        status.decode(gnss.payload());
        std::cout << "[NAV_STATUS]\n";
        std::cout << status << '\n';
        break;
      case tobas::ublox::GnssReceiver::NAV_TIMEGPS:
        timegps.decode(gnss.payload());
        std::cout << "[NAV_TIMEGPS]\n";
        std::cout << timegps << '\n';
        break;
      case tobas::ublox::GnssReceiver::NAV_VELNED:
        velned.decode(gnss.payload());
        std::cout << "[NAV_VELNED]\n";
        std::cout << velned << '\n';
        break;
      default:
        continue;
    }
  }

  return EXIT_SUCCESS;
}
