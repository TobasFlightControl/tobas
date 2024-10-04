#include <iostream>

#include <tobas_aso_core/zed_f9p.hpp>

using namespace std;

int main()
{
  aso::ZEDF9P gnss;

  aso::payload::NAV_COV cov;
  aso::payload::NAV_HPPOSLLH hpposllh;
  aso::payload::NAV_POSLLH posllh;
  aso::payload::NAV_PVT pvt;
  aso::payload::NAV_STATUS status;
  aso::payload::NAV_TIMEGPS timegps;
  aso::payload::NAV_VELNED velned;

  if (!gnss.initialize())
  {
    cerr << "Failed to initialize GNSS driver." << endl;
    return EXIT_FAILURE;
  }

  if (!gnss.configureMeasurementRate(1000))
  {
    cerr << "Failed to configure measurement rate." << endl;
    return EXIT_FAILURE;
  }

  // Enable GNSS
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

  // Enable messages
  if (!gnss.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_COV, true))
  {
    cerr << "Failed to enable NAV_COV message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_HPPOSLLH, true))
  {
    cerr << "Failed to enable NAV_HPPOSLLH message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_POSLLH, true))
  {
    cerr << "Failed to enable NAV_POSLLH message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_PVT, true))
  {
    cerr << "Failed to enable NAV_PVT message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_STATUS, true))
  {
    cerr << "Failed to enable NAV_STATUS message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_TIMEGPS, true))
  {
    cerr << "Failed to enable NAV_TIMEGPS message." << endl;
    return EXIT_FAILURE;
  }
  if (!gnss.enableMsg(aso::ZEDF9P::CLASS_NAV, aso::ZEDF9P::NAV_VELNED, true))
  {
    cerr << "Failed to enable NAV_VELNED message." << endl;
    return EXIT_FAILURE;
  }

  while (true)
  {
    if (!gnss.update(false))
    {
      cerr << "Failed to update GNSS driver." << endl;
      return EXIT_FAILURE;
    }

    if (gnss.latestClass() != aso::ZEDF9P::CLASS_NAV)
      continue;

    switch (gnss.latestId())
    {
      case aso::ZEDF9P::NAV_COV:
        cov.decode(gnss.payload());
        cout << "[NAV_COV]" << endl;
        cout << cov << endl;
        break;
      case aso::ZEDF9P::NAV_HPPOSLLH:
        hpposllh.decode(gnss.payload());
        cout << "[NAV_HPPOSLLH]" << endl;
        cout << hpposllh << endl;
        break;
      case aso::ZEDF9P::NAV_POSLLH:
        posllh.decode(gnss.payload());
        cout << "[NAV_POSLLH]" << endl;
        cout << posllh << endl;
        break;
      case aso::ZEDF9P::NAV_PVT:
        pvt.decode(gnss.payload());
        cout << "[NAV_PVT]" << endl;
        cout << pvt << endl;
        break;
      case aso::ZEDF9P::NAV_STATUS:
        status.decode(gnss.payload());
        cout << "[NAV_STATUS]" << endl;
        cout << status << endl;
        break;
      case aso::ZEDF9P::NAV_TIMEGPS:
        timegps.decode(gnss.payload());
        cout << "[NAV_TIMEGPS]" << endl;
        cout << timegps << endl;
        break;
      case aso::ZEDF9P::NAV_VELNED:
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
