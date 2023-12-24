#include "../../include/Common/ubx_payload.hpp"

using namespace std;

ostream& operator<<(ostream& os, const NavPosllhPayload& arg)
{
  os << "Longitude: " << arg.lon << "[deg]" << endl;
  os << "Latitude: " << arg.lat << "[deg]" << endl;
  os << "Height above mean sea level: " << arg.hMSL << "[m]" << endl;
  return os;
}

ostream& operator<<(ostream& os, const NavStatusPayload& arg)
{
  os << "GPSfix Type: " << static_cast<int>(arg.gpsFix) << endl;
  os << "Position and velocity valid and within DOP and ACC Masks: " << arg.gpsFixOk << endl;
  return os;
}

ostream& operator<<(ostream& os, const NavPvtPayload& arg)
{
  os << "Year (UTC): " << static_cast<int>(arg.year) << endl;
  os << "Month, range 1..12 (UTC): " << static_cast<int>(arg.month) << endl;
  os << "Day of month, range 1..31 (UTC): " << static_cast<int>(arg.day) << endl;
  os << "Hour of day, range 0..23 (UTC): " << static_cast<int>(arg.hour) << endl;
  os << "Minute of hour, range 0..59 (UTC): " << static_cast<int>(arg.min) << endl;
  os << "Seconds of minute, range 0..60 (UTC): " << static_cast<int>(arg.sec) << endl;

  os << "Valid UTC Date: " << arg.validDate << endl;
  os << "Valid UTC time of day: " << arg.validTime << endl;
  os << "UTC time of day has been fully resolved: " << arg.fullyResolved << endl;
  os << "Valid magnetic declination: " << arg.validMag << endl;

  os << "Time accuracy estimate (UTC): " << arg.tAcc << "[ns]" << endl;
  os << "Fraction of second, range -1e9 .. 1e9 (UTC): " << arg.nano << "[ns]" << endl;

  os << "GNSSfix Type: " << static_cast<int>(arg.fixType) << endl;

  os << "Valid fix (i.e within DOP & accuracy masks): " << arg.gnssFixOk << endl;

  os << "Longitude: " << arg.lon << "[deg]" << endl;
  os << "Latitude: " << arg.lat << "[deg]" << endl;
  os << "Height above mean sea level: " << arg.hMSL << "[m]" << endl;
  os << "NED north velocity: " << arg.velN << "[m/s]" << endl;
  os << "NED east velocity: " << arg.velE << "[m/s]" << endl;
  os << "NED down velocity: " << arg.velD << "[m/s]" << endl;
  return os;
}

ostream& operator<<(ostream& os, const NavVelnedPayload& arg)
{
  os << "North velocity component: " << arg.velN << "[m/s]" << endl;
  os << "East velocity component: " << arg.velE << "[m/s]" << endl;
  os << "Down velocity component: " << arg.velD << "[m/s]" << endl;
  return os;
}

ostream& operator<<(ostream& os, const NavCovPayload& arg)
{
  os << "Position covariance matrix value p_NN: " << arg.posCovNN << "[m^2]" << endl;
  os << "Position covariance matrix value p_NE: " << arg.posCovNE << "[m^2]" << endl;
  os << "Position covariance matrix value p_ND: " << arg.posCovND << "[m^2]" << endl;
  os << "Position covariance matrix value p_EE: " << arg.posCovEE << "[m^2]" << endl;
  os << "Position covariance matrix value p_ED: " << arg.posCovED << "[m^2]" << endl;
  os << "Position covariance matrix value p_DD: " << arg.posCovDD << "[m^2]" << endl;
  os << "Velocity covariance matrix value v_NN: " << arg.velCovNN << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_NE: " << arg.velCovNE << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_ND: " << arg.velCovND << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_EE: " << arg.velCovEE << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_ED: " << arg.velCovED << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_DD: " << arg.velCovDD << "[m^2/s^2]" << endl;
  return os;
}
