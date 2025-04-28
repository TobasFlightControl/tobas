#include <iostream>

#include <tobas_ic_drivers/ublox/zed_f9p_1xb.hpp>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <SPI Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];

  ublox::ZEDF9P1xB gnss;

  ublox::payload::NAV_COV cov;
  ublox::payload::NAV_HPPOSLLH hpposllh;
  ublox::payload::NAV_POSLLH posllh;
  ublox::payload::NAV_PVT pvt;
  ublox::payload::NAV_STATUS status;
  ublox::payload::NAV_TIMEGPS timegps;
  ublox::payload::NAV_VELNED velned;

  cout << "Initializing GNSS device." << endl;
  if (!gnss.initialize(device))
  {
    cerr << "Failed to initialize GNSS driver." << endl;
    return EXIT_FAILURE;
  }

  cout << "Configuring measurement rate." << endl;
  if (!gnss.configureMeasurementRate(1000))
  {
    cerr << "Failed to configure measurement rate." << endl;
    return EXIT_FAILURE;
  }

  // Enable GNSS
  cout << "Enabling GNSS." << endl;
  if (!gnss.enableGPS(true))
  {
    cerr << "Failed to enable GPS." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableSBAS(true))
  {
    cerr << "Failed to enable SBAS." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableGalileo(false))
  {
    cerr << "Failed to disable Galileo." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableBeiDou(false))
  {
    cerr << "Failed to disable BeiDou." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableQZSS(true))
  {
    cerr << "Failed to enable QZSS." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableGLONASS(false))
  {
    cerr << "Failed to disable GLONASS." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableNavIC(false))
  {
    cerr << "Failed to disable NavIC." << endl;
    return EXIT_FAILURE;
  }

  // Enable messages
  cout << "Enabling messages." << endl;
  if (!gnss.enableMsg(ublox::ZEDF9P1xB::CLASS_NAV, ublox::ZEDF9P1xB::NAV_COV, true))
  {
    cerr << "Failed to enable NAV_COV message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(ublox::ZEDF9P1xB::CLASS_NAV, ublox::ZEDF9P1xB::NAV_HPPOSLLH, true))
  {
    cerr << "Failed to enable NAV_HPPOSLLH message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(ublox::ZEDF9P1xB::CLASS_NAV, ublox::ZEDF9P1xB::NAV_POSLLH, true))
  {
    cerr << "Failed to enable NAV_POSLLH message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(ublox::ZEDF9P1xB::CLASS_NAV, ublox::ZEDF9P1xB::NAV_PVT, true))
  {
    cerr << "Failed to enable NAV_PVT message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(ublox::ZEDF9P1xB::CLASS_NAV, ublox::ZEDF9P1xB::NAV_STATUS, true))
  {
    cerr << "Failed to enable NAV_STATUS message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(ublox::ZEDF9P1xB::CLASS_NAV, ublox::ZEDF9P1xB::NAV_TIMEGPS, true))
  {
    cerr << "Failed to enable NAV_TIMEGPS message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(ublox::ZEDF9P1xB::CLASS_NAV, ublox::ZEDF9P1xB::NAV_VELNED, true))
  {
    cerr << "Failed to enable NAV_VELNED message." << endl;
    return EXIT_FAILURE;
  }

  cout << "Initial configuration is finished successfully." << endl;

  while (true)
  {
    if (!gnss.update(false))
    {
      cerr << "Failed to update GNSS driver." << endl;
      return EXIT_FAILURE;
    }

    if (gnss.latestClass() != ublox::ZEDF9P1xB::CLASS_NAV)
      continue;

    switch (gnss.latestId())
    {
      case ublox::ZEDF9P1xB::NAV_COV:
        cov.decode(gnss.payload());
        cout << "[NAV_COV]" << endl;
        cout << cov << endl;
        break;
      case ublox::ZEDF9P1xB::NAV_HPPOSLLH:
        hpposllh.decode(gnss.payload());
        cout << "[NAV_HPPOSLLH]" << endl;
        cout << hpposllh << endl;
        break;
      case ublox::ZEDF9P1xB::NAV_POSLLH:
        posllh.decode(gnss.payload());
        cout << "[NAV_POSLLH]" << endl;
        cout << posllh << endl;
        break;
      case ublox::ZEDF9P1xB::NAV_PVT:
        pvt.decode(gnss.payload());
        cout << "[NAV_PVT]" << endl;
        cout << pvt << endl;
        break;
      case ublox::ZEDF9P1xB::NAV_STATUS:
        status.decode(gnss.payload());
        cout << "[NAV_STATUS]" << endl;
        cout << status << endl;
        break;
      case ublox::ZEDF9P1xB::NAV_TIMEGPS:
        timegps.decode(gnss.payload());
        cout << "[NAV_TIMEGPS]" << endl;
        cout << timegps << endl;
        break;
      case ublox::ZEDF9P1xB::NAV_VELNED:
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
