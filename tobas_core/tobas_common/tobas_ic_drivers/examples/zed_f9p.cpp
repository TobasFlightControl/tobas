// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>

#include <tobas_ic_drivers/ublox/ubx_transport_spi.hpp>
#include <tobas_ic_drivers/ublox/ubx_transport_uart.hpp>
#include <tobas_ic_drivers/ublox/zed_f9p.hpp>

using namespace std;

int main(int _argc, char** _argv)
{
  unique_ptr<tobas::ublox::UbxTransport> transport;

  switch (_argc) {
    case 2:
      transport = make_unique<tobas::ublox::UbxTransportSpi>(_argv[1]);
      break;
    case 3:
      transport = make_unique<tobas::ublox::UbxTransportUart>(_argv[1], stoi(_argv[2]));
      break;
    default:
      cerr << "Usage 1: " << _argv[0] << " <SPI Device>\n";
      cerr << "Usage 2: " << _argv[0] << " <UART Device> <Baud Rate>\n";
      return EXIT_FAILURE;
  }

  tobas::ublox::ZEDF9P gnss(move(transport));

  tobas::ublox::payload::NAV_COV cov;
  tobas::ublox::payload::NAV_HPPOSLLH hpposllh;
  tobas::ublox::payload::NAV_POSLLH posllh;
  tobas::ublox::payload::NAV_PVT pvt;
  tobas::ublox::payload::NAV_SAT sat;
  tobas::ublox::payload::NAV_STATUS status;
  tobas::ublox::payload::NAV_TIMEGPS timegps;
  tobas::ublox::payload::NAV_VELNED velned;

  cout << "Initializing GNSS device.\n";
  if (!gnss.initialize()) {
    cerr << "Failed to initialize GNSS driver.\n";
    return EXIT_FAILURE;
  }

  cout << "Configuring measurement rate.\n";
  if (!gnss.configureMeasurementRate(1000)) {
    cerr << "Failed to configure measurement rate.\n";
    return EXIT_FAILURE;
  }

  // Enable GNSS.
  cout << "Enabling GNSS.\n";
  if (!gnss.enableGps()) {
    cerr << "Failed to enable GPS.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSbas()) {
    cerr << "Failed to enable SBAS.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.disableGalileo()) {
    cerr << "Failed to disable Galileo.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.disableBeiDou()) {
    cerr << "Failed to disable BeiDou.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableQzss()) {
    cerr << "Failed to enable QZSS.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.disableGlonass()) {
    cerr << "Failed to disable GLONASS.\n";
    return EXIT_FAILURE;
  }

  // Enable messages.
  cout << "Enabling messages.\n";
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_COV, true)) {
    cerr << "Failed to enable NAV_COV message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_HPPOSLLH, true)) {
    cerr << "Failed to enable NAV_HPPOSLLH message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_POSLLH, true)) {
    cerr << "Failed to enable NAV_POSLLH message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_PVT, true)) {
    cerr << "Failed to enable NAV_PVT message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_SAT, true)) {
    cerr << "Failed to enable NAV_SAT message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_STATUS, true)) {
    cerr << "Failed to enable NAV_STATUS message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_TIMEGPS, true)) {
    cerr << "Failed to enable NAV_TIMEGPS message.\n";
    return EXIT_FAILURE;
  }
  if (!gnss.enableSpiMessage(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_VELNED, true)) {
    cerr << "Failed to enable NAV_VELNED message.\n";
    return EXIT_FAILURE;
  }

  cout << "Initial configuration finished successfully.\n";

  while (true) {
    if (!gnss.update(true)) {
      cerr << "Failed to receive a GNSS message.\n";
      return EXIT_FAILURE;
    }

    if (gnss.latestClass() != tobas::ublox::ZEDF9P::CLASS_NAV) {
      continue;
    }

    switch (gnss.latestId()) {
      case tobas::ublox::ZEDF9P::NAV_COV:
        cov.decode(gnss.payload());
        cout << "[NAV_COV]\n";
        cout << cov << '\n';
        break;
      case tobas::ublox::ZEDF9P::NAV_HPPOSLLH:
        hpposllh.decode(gnss.payload());
        cout << "[NAV_HPPOSLLH]\n";
        cout << hpposllh << '\n';
        break;
      case tobas::ublox::ZEDF9P::NAV_POSLLH:
        posllh.decode(gnss.payload());
        cout << "[NAV_POSLLH]\n";
        cout << posllh << '\n';
        break;
      case tobas::ublox::ZEDF9P::NAV_PVT:
        pvt.decode(gnss.payload());
        cout << "[NAV_PVT]\n";
        cout << pvt << '\n';
        break;
      case tobas::ublox::ZEDF9P::NAV_SAT:
        sat.decode(gnss.payload());
        cout << "[NAV_SAT]\n";
        cout << sat << '\n';
        break;
      case tobas::ublox::ZEDF9P::NAV_STATUS:
        status.decode(gnss.payload());
        cout << "[NAV_STATUS]\n";
        cout << status << '\n';
        break;
      case tobas::ublox::ZEDF9P::NAV_TIMEGPS:
        timegps.decode(gnss.payload());
        cout << "[NAV_TIMEGPS]\n";
        cout << timegps << '\n';
        break;
      case tobas::ublox::ZEDF9P::NAV_VELNED:
        velned.decode(gnss.payload());
        cout << "[NAV_VELNED]\n";
        cout << velned << '\n';
        break;
      default:
        continue;
    }
  }

  return EXIT_SUCCESS;
}
