#include <vector>
#include <iostream>
#include <unistd.h>

#include <tobas_navio_core/neo_m8n.hpp>
#include <tobas_navio_core/util.hpp>

#define MEASUREMENT_RATE 100  // [ms]
#define SLEEP_TIME 200        // [us]

using namespace std;
using namespace navio;

int main()
{
  if (checkAPM())
    return 1;

  // NEOM8N class instance
  NEOM8N gps;

  // Payloads
  NavPosllhPayload posllh;
  NavStatusPayload status;
  NavPvtPayload pvt;
  NavVelnedPayload velned;
  NavCovPayload cov;

  // Set message rate
  gps.enableMsg(NEOM8N::NAV_POSLLH, false);
  gps.enableMsg(NEOM8N::NAV_STATUS, true);
  gps.enableMsg(NEOM8N::NAV_PVT, true);
  gps.enableMsg(NEOM8N::NAV_VELNED, false);
  gps.enableMsg(NEOM8N::NAV_COV, true);

  // Navigation/measurement rate settings
  gps.configureSolutionRate(MEASUREMENT_RATE);

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
      case NEOM8N::NAV_POSLLH:
        gps.decode(posllh);
        cout << "NAV_POSLLH(" << ++cnt_posllh << "):" << endl << posllh << endl;
        break;
      case NEOM8N::NAV_STATUS:
        gps.decode(status);
        cout << "NAV_STATUS(" << ++cnt_status << "):" << endl << status << endl;
        break;
      case NEOM8N::NAV_PVT:
        gps.decode(pvt);
        cout << "NAV_PVT(" << ++cnt_pvt << "):" << endl << pvt << endl;
        break;
      case NEOM8N::NAV_VELNED:
        gps.decode(velned);
        cout << "NAV_VELNED(" << ++cnt_velned << "):" << endl << velned << endl;
        break;
      case NEOM8N::NAV_COV:
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
