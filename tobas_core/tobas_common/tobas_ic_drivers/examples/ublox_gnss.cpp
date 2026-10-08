// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <iostream>


#include <tobas_ic_drivers/ublox/ubx_transport_spi.hpp>
#include <tobas_ic_drivers/ublox/ubx_transport_uart.hpp>
#include <tobas_ic_drivers/ublox/ublox_gnss.hpp>

using namespace std;

int main(int argc, char** argv)
{
  constexpr uint32_t kInitialUartBaudRate = 38400;
  constexpr uint32_t kUartBaudRate = 230400;

  unique_ptr<tobas::ublox::UbxTransport> transport;
  tobas::ublox::UbxTransportUart* uart_transport = nullptr;
  bool use_uart = false;

  if (argc == 3 && string(argv[1]) == "spi") {
    transport = make_unique<tobas::ublox::UbxTransportSpi>(argv[2]);
  }
  else if (argc == 3 && string(argv[1]) == "uart") {
    use_uart = true;

    auto uart = make_unique<tobas::ublox::UbxTransportUart>(
      argv[2], kInitialUartBaudRate);

    uart_transport = uart.get();
    transport = move(uart);
  }
  else {
    cerr << "Usage 1: " << argv[0]
         << " spi <SPI Device>" << endl;
    cerr << "Usage 2: " << argv[0]
         << " uart <UART Device>" << endl;
    return EXIT_FAILURE;
  }

  tobas::ublox::UbloxGnss gnss(move(transport));

  tobas::ublox::payload::NAV_COV cov;
  tobas::ublox::payload::NAV_HPPOSLLH hpposllh;
  tobas::ublox::payload::NAV_POSLLH posllh;
  tobas::ublox::payload::NAV_PVT pvt;
  tobas::ublox::payload::NAV_SAT sat;
  tobas::ublox::payload::NAV_STATUS status;
  tobas::ublox::payload::NAV_TIMEGPS timegps;
  tobas::ublox::payload::NAV_VELNED velned;

  cout << "Initializing GNSS device." << endl;
  if (!gnss.initialize()) {
    cerr << "Failed to initialize GNSS driver." << endl;
    return EXIT_FAILURE;
  }

  if (use_uart) {
    if (!uart_transport->setReceiveTimeout(10)) {
      cerr << "Failed to set UART receive timeout." << endl;
      return EXIT_FAILURE;
    }
  }

  auto module_name = gnss.getModuleName();

  if (!module_name && use_uart) {
    cout << "No response at " << kInitialUartBaudRate
         << " baud. Trying " << kUartBaudRate << " baud." << endl;

    if (!uart_transport->setBaudRate(kUartBaudRate)) {
      cerr << "Failed to change host UART baud rate." << endl;
      return EXIT_FAILURE;
    }

    module_name = gnss.getModuleName();
  }

  if (!module_name) {
    cerr << "Failed to detect GNSS module." << endl;
    return EXIT_FAILURE;
  }

  cout << "Detected module: " << *module_name << endl;

  if (use_uart) {
    if (!uart_transport->disableReceiveTimeout()) {
      cerr << "Failed to disable UART receive timeout." << endl;
      return EXIT_FAILURE;
    }
  }

  if (use_uart) {
    if (!gnss.configureUartBaudRate(kUartBaudRate)) {
      cerr << "Failed to configure UART baud rate." << endl;
      return EXIT_FAILURE;
    }

    if (!uart_transport->setBaudRate(kUartBaudRate)) {
      cerr << "Failed to change host UART baud rate." << endl;
      return EXIT_FAILURE;
    }
  }

  uint16_t measurement_period_ms;

  if (*module_name == "ZED-F9P") {
    gnss.setReceiverProfile(tobas::ublox::UbloxGnss::F9P);
    measurement_period_ms = 100;
  }
  else if (*module_name == "ZED-X20P") {
    gnss.setReceiverProfile(tobas::ublox::UbloxGnss::X20P);
    measurement_period_ms = 40;
  }
  else {
    cerr << "Unsupported GNSS module: " << *module_name << endl;
    return EXIT_FAILURE;
  }

  cout << "Configuring measurement rate." << endl;
  if (!gnss.configureMeasurementRate(measurement_period_ms)) {
    cerr << "Failed to configure measurement rate." << endl;
    return EXIT_FAILURE;
  }

  // Enable GNSS.
  cout << "Enabling GNSS." << endl;
  if (!gnss.enableGps()) {
    cerr << "Failed to enable GPS." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableSbas()) {
    cerr << "Failed to enable SBAS." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.disableGalileo()) {
    cerr << "Failed to disable Galileo." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.disableBeiDou()) {
    cerr << "Failed to disable BeiDou." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableQzss()) {
    cerr << "Failed to enable QZSS." << endl;
    return EXIT_FAILURE;
  }

  if (*module_name == "ZED-F9P") {
    if (!gnss.disableGlonass()) {
      cerr << "Failed to disable GLONASS." << endl;
      return EXIT_FAILURE;
    }
  }
  else if (*module_name == "ZED-X20P") {
    if (!gnss.enableNavIc()) {
      cerr << "Failed to enable NavIC." << endl;
      return EXIT_FAILURE;
    }
  }

  // Enable messages.
  cout << "Enabling messages." << endl;
  if (use_uart) {
    if (!gnss.enableUartMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_COV, true)) {
      cerr << "Failed to enable NAV_COV message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableUartMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_HPPOSLLH, true)) {
      cerr << "Failed to enable NAV_HPPOSLLH message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableUartMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_POSLLH, true)) {
      cerr << "Failed to enable NAV_POSLLH message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableUartMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_PVT, true)) {
      cerr << "Failed to enable NAV_PVT message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableUartMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_SAT, true)) {
      cerr << "Failed to enable NAV_SAT message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableUartMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_STATUS, true)) {
      cerr << "Failed to enable NAV_STATUS message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableUartMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_TIMEGPS, true)) {
      cerr << "Failed to enable NAV_TIMEGPS message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableUartMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_VELNED, true)) {
      cerr << "Failed to enable NAV_VELNED message." << endl;
      return EXIT_FAILURE;
    }
  } else {
    if (!gnss.enableSpiMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_COV, true)) {
      cerr << "Failed to enable NAV_COV message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableSpiMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_HPPOSLLH, true)) {
      cerr << "Failed to enable NAV_HPPOSLLH message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableSpiMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_POSLLH, true)) {
      cerr << "Failed to enable NAV_POSLLH message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableSpiMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_PVT, true)) {
      cerr << "Failed to enable NAV_PVT message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableSpiMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_SAT, true)) {
      cerr << "Failed to enable NAV_SAT message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableSpiMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_STATUS, true)) {
      cerr << "Failed to enable NAV_STATUS message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableSpiMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_TIMEGPS, true)) {
      cerr << "Failed to enable NAV_TIMEGPS message." << endl;
      return EXIT_FAILURE;
    }
    if (!gnss.enableSpiMessage(tobas::ublox::UbloxGnss::CLASS_NAV, tobas::ublox::UbloxGnss::NAV_VELNED, true)) {
      cerr << "Failed to enable NAV_VELNED message." << endl;
      return EXIT_FAILURE;
    }
  }

  cout << "Initial configuration finished successfully." << endl;

  while (true) {
    if (!gnss.update(true)) {
      cerr << "Failed to receive a GNSS message." << endl;
      return EXIT_FAILURE;
    }

    if (gnss.latestClass() != tobas::ublox::UbloxGnss::CLASS_NAV) {
      continue;
    }

    switch (gnss.latestId()) {
      case tobas::ublox::UbloxGnss::NAV_COV:
        cov.decode(gnss.payload());
        cout << "[NAV_COV]" << endl;
        cout << cov << endl;
        break;
      case tobas::ublox::UbloxGnss::NAV_HPPOSLLH:
        hpposllh.decode(gnss.payload());
        cout << "[NAV_HPPOSLLH]" << endl;
        cout << hpposllh << endl;
        break;
      case tobas::ublox::UbloxGnss::NAV_POSLLH:
        posllh.decode(gnss.payload());
        cout << "[NAV_POSLLH]" << endl;
        cout << posllh << endl;
        break;
      case tobas::ublox::UbloxGnss::NAV_PVT:
        pvt.decode(gnss.payload());
        cout << "[NAV_PVT]" << endl;
        cout << pvt << endl;
        break;
      case tobas::ublox::UbloxGnss::NAV_SAT:
        sat.decode(gnss.payload());
        cout << "[NAV_SAT]" << endl;
        cout << sat << endl;
        break;
      case tobas::ublox::UbloxGnss::NAV_STATUS:
        status.decode(gnss.payload());
        cout << "[NAV_STATUS]" << endl;
        cout << status << endl;
        break;
      case tobas::ublox::UbloxGnss::NAV_TIMEGPS:
        timegps.decode(gnss.payload());
        cout << "[NAV_TIMEGPS]" << endl;
        cout << timegps << endl;
        break;
      case tobas::ublox::UbloxGnss::NAV_VELNED:
        velned.decode(gnss.payload());
        cout << "[NAV_VELNED]" << endl;
        cout << velned << endl;
        break;
      default:
        continue;
    }
  }

  return EXIT_SUCCESS;
}
