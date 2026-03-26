#include <iostream>

#include <tobas_ic_drivers/ublox/zed_f9p.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <SPI Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];

  tobas::ublox::ZEDF9P gnss;

  tobas::ublox::payload::NAV_COV cov;
  tobas::ublox::payload::NAV_HPPOSLLH hpposllh;
  tobas::ublox::payload::NAV_POSLLH posllh;
  tobas::ublox::payload::NAV_PVT pvt;
  tobas::ublox::payload::NAV_STATUS status;
  tobas::ublox::payload::NAV_TIMEGPS timegps;
  tobas::ublox::payload::NAV_VELNED velned;

  cout << "Initializing GNSS device." << endl;
  if (!gnss.initialize(device)) {
    cerr << "Failed to initialize GNSS driver." << endl;
    return EXIT_FAILURE;
  }

  cout << "Configuring measurement rate." << endl;
  if (!gnss.configureMeasurementRate(1000)) {
    cerr << "Failed to configure measurement rate." << endl;
    return EXIT_FAILURE;
  }

  // Enable GNSS
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
  if (!gnss.disableGlonass()) {
    cerr << "Failed to disable GLONASS." << endl;
    return EXIT_FAILURE;
  }

  // Enable messages
  cout << "Enabling messages." << endl;
  if (!gnss.enableMsg(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_COV, true)) {
    cerr << "Failed to enable NAV_COV message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_HPPOSLLH, true)) {
    cerr << "Failed to enable NAV_HPPOSLLH message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_POSLLH, true)) {
    cerr << "Failed to enable NAV_POSLLH message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_PVT, true)) {
    cerr << "Failed to enable NAV_PVT message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_STATUS, true)) {
    cerr << "Failed to enable NAV_STATUS message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_TIMEGPS, true)) {
    cerr << "Failed to enable NAV_TIMEGPS message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(tobas::ublox::ZEDF9P::CLASS_NAV, tobas::ublox::ZEDF9P::NAV_VELNED, true)) {
    cerr << "Failed to enable NAV_VELNED message." << endl;
    return EXIT_FAILURE;
  }

  cout << "Initial configuration finished successfully." << endl;

  while (true) {
    if (!gnss.update(false)) {
      cerr << "Failed to update GNSS driver." << endl;
      return EXIT_FAILURE;
    }

    if (gnss.latestClass() != tobas::ublox::ZEDF9P::CLASS_NAV) {
      continue;
    }

    switch (gnss.latestId()) {
      case tobas::ublox::ZEDF9P::NAV_COV:
        cov.decode(gnss.payload());
        cout << "[NAV_COV]" << endl;
        cout << cov << endl;
        break;
      case tobas::ublox::ZEDF9P::NAV_HPPOSLLH:
        hpposllh.decode(gnss.payload());
        cout << "[NAV_HPPOSLLH]" << endl;
        cout << hpposllh << endl;
        break;
      case tobas::ublox::ZEDF9P::NAV_POSLLH:
        posllh.decode(gnss.payload());
        cout << "[NAV_POSLLH]" << endl;
        cout << posllh << endl;
        break;
      case tobas::ublox::ZEDF9P::NAV_PVT:
        pvt.decode(gnss.payload());
        cout << "[NAV_PVT]" << endl;
        cout << pvt << endl;
        break;
      case tobas::ublox::ZEDF9P::NAV_STATUS:
        status.decode(gnss.payload());
        cout << "[NAV_STATUS]" << endl;
        cout << status << endl;
        break;
      case tobas::ublox::ZEDF9P::NAV_TIMEGPS:
        timegps.decode(gnss.payload());
        cout << "[NAV_TIMEGPS]" << endl;
        cout << timegps << endl;
        break;
      case tobas::ublox::ZEDF9P::NAV_VELNED:
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
