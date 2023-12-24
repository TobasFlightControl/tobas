#include <vector>
#include <iostream>

#include <Common/Ublox.h>
#include <Common/Util.h>

#define MEASUREMENT_RATE 100  // [ms]
#define SLEEP_TIME 200        // [us]

using namespace std;

int main()
{
  if (check_apm())
  {
    return 1;
  }

  // Ublox class instance
  Ublox gps;

  // Payloads
  NavPosllhPayload posllh;
  NavStatusPayload status;
  NavPvtPayload pvt;
  NavVelnedPayload velned;
  NavCovPayload cov;

  // Set message rate
  gps.enableMsg(Ublox::NAV_POSLLH, false);
  gps.enableMsg(Ublox::NAV_STATUS, true);
  gps.enableMsg(Ublox::NAV_PVT, true);
  gps.enableMsg(Ublox::NAV_VELNED, false);
  gps.enableMsg(Ublox::NAV_COV, true);

  // Navigation/measurement rate settings
  if (!gps.configureSolutionRate(MEASUREMENT_RATE))
  {
    cerr << "Failed to configure solution rate." << endl;
    return 1;
  }

  uint32_t cnt_posllh = 0;
  uint32_t cnt_status = 0;
  uint32_t cnt_pvt = 0;
  uint32_t cnt_velned = 0;
  uint32_t cnt_cov = 0;

  while (true)
  {
    const auto msg_id = gps.update();

    switch (msg_id)
    {
      case Ublox::NAV_POSLLH:
        gps.decode(posllh);
        cout << "NAV_POSLLH(" << ++cnt_posllh << "):" << endl << posllh << endl;
        break;
      case Ublox::NAV_STATUS:
        gps.decode(status);
        cout << "NAV_STATUS(" << ++cnt_status << "):" << endl << status << endl;
        break;
      case Ublox::NAV_PVT:
        gps.decode(pvt);
        cout << "NAV_PVT(" << ++cnt_pvt << "):" << endl << pvt << endl;
        break;
      case Ublox::NAV_VELNED:
        gps.decode(velned);
        cout << "NAV_VELNED(" << ++cnt_velned << "):" << endl << velned << endl;
        break;
      case Ublox::NAV_COV:
        gps.decode(cov);
        cout << "NAV_COV(" << ++cnt_cov << "):" << endl << cov << endl;
        break;
      default:
        break;
    }

    usleep(SLEEP_TIME);
  }

  return 0;
}
