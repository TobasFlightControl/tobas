#include "../include/tobas_a1_core/ubx_payload.hpp"

using namespace std;

namespace a1
{
namespace payload
{
ostream& operator<<(ostream& os, const NAV_POSLLH& arg)
{
  os << "GPS time of week: " << arg.iTOW << "[ms]" << endl;

  os << "Longitude: " << arg.lon << "[deg]" << endl;
  os << "Latitude: " << arg.lat << "[deg]" << endl;
  os << "Height above ellipsoid: " << arg.height << "[mm]" << endl;
  os << "Height above mean sea level: " << arg.hMSL << "[mm]" << endl;

  os << "Horizontal accuracy estimate: " << arg.hAcc << "[mm]" << endl;
  os << "Vertical accuracy estimate: " << arg.vAcc << "[mm]" << endl;

  return os;
}

ostream& operator<<(ostream& os, const NAV_PVT& arg)
{
  os << "GPS time of week: " << arg.iTOW << "[ms]" << endl;

  os << "Year (UTC): " << (int)arg.year << endl;
  os << "Month, range 1..12 (UTC): " << (int)arg.month << endl;
  os << "Day of month, range 1..31 (UTC): " << (int)arg.day << endl;
  os << "Hour of day, range 0..23 (UTC): " << (int)arg.hour << endl;
  os << "Minute of hour, range 0..59 (UTC): " << (int)arg.min << endl;
  os << "Seconds of minute, range 0..60 (UTC): " << (int)arg.sec << endl;

  os << "Valid UTC Date: " << arg.validDate << endl;
  os << "Valid UTC time of day: " << arg.validTime << endl;
  os << "UTC time of day has been fully resolved: " << arg.fullyResolved << endl;
  os << "Valid magnetic declination: " << arg.validMag << endl;

  os << "Time accuracy estimate (UTC): " << arg.tAcc << "[ns]" << endl;
  os << "Fraction of second, range -1e9 .. 1e9 (UTC): " << arg.nano << "[ns]" << endl;

  os << "GNSSfix Type: " << (int)arg.fixType << endl;

  os << "Valid fix (i.e within DOP & accuracy masks): " << arg.gnssFixOk << endl;
  os << "Differential corrections were applied: " << arg.diffSoln << endl;
  os << "Power Save Mode state: " << (int)arg.psmState << endl;
  os << "Heading of vehicle is valid: " << arg.headVehValid << endl;
  os << "Carrier phase range solution status: " << arg.carrSoln << endl;

  os << "Information about UTC Date and Time of Day validity confirmation is available: " << arg.confirmedAvai << endl;
  os << "UTC Date validity could be confirmed: " << arg.confirmedDate << endl;
  os << "UTC Time of Day could be confirmed: " << arg.confirmedTime << endl;

  os << "Longitude: " << arg.lon << "[deg]" << endl;
  os << "Latitude: " << arg.lat << "[deg]" << endl;
  os << "Height above ellipsoid: " << arg.height << "[mm]" << endl;
  os << "Height above mean sea level: " << arg.hMSL << "[mm]" << endl;
  os << "Horizontal accuracy estimate: " << arg.hAcc << "[mm]" << endl;
  os << "Vertical accuracy estimate: " << arg.vAcc << "[mm]" << endl;
  os << "NED north velocity: " << arg.velN << "[m/s]" << endl;
  os << "NED east velocity: " << arg.velE << "[m/s]" << endl;
  os << "NED down velocity: " << arg.velD << "[m/s]" << endl;
  os << "Ground Speed (2-D): " << arg.gSpeed << "[mm/s]" << endl;
  os << "Heading of motion (2-D): " << arg.headMot << "[deg]" << endl;
  os << "Speed accuracy estimate: " << arg.sAcc << "[mm/s]" << endl;
  os << "Heading accuracy estimate: " << arg.headAcc << "[deg]" << endl;
  os << "Position DOP: " << arg.pDOP << endl;

  os << "Invalid lon, lat, height and hMSL: " << arg.invalidLlh << endl;

  os << "Heading of vehicle (2-D): " << arg.headVeh << "[deg]" << endl;
  os << "Magnetic declination: " << arg.magDec << "[deg]" << endl;
  os << "Magnetic declination accuracy: " << arg.magAcc << "[deg]" << endl;

  return os;
}

ostream& operator<<(ostream& os, const NAV_STATUS& arg)
{
  os << "GPS time of week: " << arg.iTOW << "[ms]" << endl;
  os << "GPSfix Type: " << (int)arg.gpsFix << endl;

  os << "Position and velocity valid and within DOP and ACC Masks: " << arg.gpsFixOk << endl;
  os << "Differential corrections were applied: " << arg.diffSoln << endl;
  os << "Week Number valid: " << arg.wknSet << endl;
  os << "Time of Week valid: " << arg.towSet << endl;

  os << "Differential corrections available: " << arg.diffCorr << endl;
  os << "Map matching status: " << (int)arg.mapMatching << endl;

  os << "Power save mode state: " << (int)arg.psmState << endl;
  os << "Spoofing detection state: " << (int)arg.spoofDetState << endl;

  os << "Time to first fix: " << arg.ttff << "[ms]" << endl;
  os << "Milliseconds since Startup / Reset: " << arg.msss << "[ms]" << endl;

  return os;
}

ostream& operator<<(ostream& os, const NAV_VELNED& arg)
{
  os << "GPS time of week: " << arg.iTOW << "[ms]" << endl;

  os << "North velocity component: " << arg.velN << "[cm/s]" << endl;
  os << "East velocity component: " << arg.velE << "[cm/s]" << endl;
  os << "Down velocity component: " << arg.velD << "[cm/s]" << endl;

  os << "Speed (3-D): " << arg.speed << "[cm/s]" << endl;
  os << "Ground Speed (3-D): " << arg.gSpeed << "[cm/s]" << endl;
  os << "Heading of motion 2-D: " << arg.heading << "[deg]" << endl;

  os << "Speed accuracy estimate: " << arg.sAcc << "[mm/s]" << endl;
  os << "Course / Heading accuracy estimate: " << arg.cAcc << "[deg]" << endl;

  return os;
}
}  // namespace payload
}  // namespace a1
