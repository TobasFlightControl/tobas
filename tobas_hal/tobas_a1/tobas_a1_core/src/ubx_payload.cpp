#include <tobas_algorithm/binary.hpp>

#include "../include/tobas_a1_core/ubx_payload.hpp"

using namespace std;

namespace a1
{
namespace payload
{
/* ===== Decode Methods ===== */

void ACK_NAK::decode(const uint8_t* p)
{
  clsID = algo::decodeU8(p + 0);
  msgID = algo::decodeU8(p + 1);
}

void ACK_ACK::decode(const uint8_t* p)
{
  clsID = algo::decodeU8(p + 0);
  msgID = algo::decodeU8(p + 1);
}

void NAV_CLOCK::decode(const uint8_t*)
{
  // TODO
}

void NAV_COV::decode(const uint8_t* p)
{
  iTOW = algo::decodeU32(p + 0);
  version = algo::decodeU8(p + 4);
  posCovValid = algo::decodeU8(p + 5);
  velCovValid = algo::decodeU8(p + 6);

  posCovNN = algo::decodeR32(p + 16);
  posCovNE = algo::decodeR32(p + 20);
  posCovND = algo::decodeR32(p + 24);
  posCovEE = algo::decodeR32(p + 28);
  posCovED = algo::decodeR32(p + 32);
  posCovDD = algo::decodeR32(p + 36);
  velCovNN = algo::decodeR32(p + 40);
  velCovNE = algo::decodeR32(p + 44);
  velCovND = algo::decodeR32(p + 48);
  velCovEE = algo::decodeR32(p + 52);
  velCovED = algo::decodeR32(p + 56);
  velCovDD = algo::decodeR32(p + 60);
}

void NAV_DOP::decode(const uint8_t*)
{
  // TODO
}

void NAV_EOE::decode(const uint8_t*)
{
  // TODO
}

void NAV_GEOFENCE::decode(const uint8_t*)
{
  // TODO
}

void NAV_HPPOSECEF::decode(const uint8_t*)
{
  // TODO
}

void NAV_HPPOSLLH::decode(const uint8_t* p)
{
  version = algo::decodeU8(p + 0);

  const auto flags = algo::decodeU8(p + 3);
  invalidLlh = (flags >> 0) & 1;

  iTOW = algo::decodeU32(p + 4);

  lon = algo::decodeI32(p + 8) * 1e-7;
  lat = algo::decodeI32(p + 12) * 1e-7;
  height = algo::decodeI32(p + 16);
  hMSL = algo::decodeI32(p + 20);

  lonHp = algo::decodeI8(p + 24) * 1e-9;
  latHp = algo::decodeI8(p + 25) * 1e-9;
  heightHp = algo::decodeI8(p + 26) * 0.1;
  hMSLHp = algo::decodeI8(p + 27) * 0.1;

  hAcc = algo::decodeU32(p + 28) * 0.1;
  vAcc = algo::decodeU32(p + 32) * 0.1;
}

void NAV_ODO::decode(const uint8_t*)
{
  // TODO
}

void NAV_ORB::decode(const uint8_t*)
{
  // TODO
}

void NAV_PL::decode(const uint8_t*)
{
  // TODO
}

void NAV_POSECEF::decode(const uint8_t*)
{
  // TODO
}

void NAV_POSLLH::decode(const uint8_t* p)
{
  iTOW = algo::decodeU32(p + 0);

  lon = algo::decodeI32(p + 4) * 1e-7;
  lat = algo::decodeI32(p + 8) * 1e-7;
  height = algo::decodeI32(p + 12);
  hMSL = algo::decodeI32(p + 16);

  hAcc = algo::decodeU32(p + 20);
  vAcc = algo::decodeU32(p + 24);
}

void NAV_PVT::decode(const uint8_t* p)
{
  iTOW = algo::decodeU32(p + 0);

  year = algo::decodeU16(p + 4);
  month = algo::decodeU8(p + 6);
  day = algo::decodeU8(p + 7);
  hour = algo::decodeU8(p + 8);
  min = algo::decodeU8(p + 9);
  sec = algo::decodeU8(p + 10);

  const auto valid = algo::decodeU8(p + 11);
  validDate = (valid >> 0) & 1;
  validTime = (valid >> 1) & 1;
  fullyResolved = (valid >> 2) & 1;
  validMag = (valid >> 3) & 1;

  tAcc = algo::decodeU32(p + 12);
  nano = algo::decodeI32(p + 16);

  fixType = static_cast<fix_type_t>(algo::decodeU8(p + 20));

  const auto flags = algo::decodeU8(p + 21);
  gnssFixOk = (flags >> 0) & 1;
  diffSoln = (flags >> 1) & 1;
  psmState = static_cast<payload::NAV_PVT::psm_state_t>((flags >> 2) & 0b111);
  headVehValid = (flags >> 5) & 1;
  carrSoln = static_cast<payload::NAV_PVT::carr_soln_t>((flags >> 6) & 0b11);

  const auto flags2 = algo::decodeU8(p + 22);
  confirmedAvai = (flags2 >> 5) & 1;
  confirmedDate = (flags2 >> 6) & 1;
  confirmedTime = (flags2 >> 7) & 1;

  numSV = algo::decodeU8(p + 23);

  lon = algo::decodeI32(p + 24) * 1e-7;
  lat = algo::decodeI32(p + 28) * 1e-7;
  height = algo::decodeI32(p + 32);
  hMSL = algo::decodeI32(p + 36);
  hAcc = algo::decodeU32(p + 40);
  vAcc = algo::decodeU32(p + 44);
  velN = algo::decodeI32(p + 48);
  velE = algo::decodeI32(p + 52);
  velD = algo::decodeI32(p + 56);
  gSpeed = algo::decodeI32(p + 60);
  headMot = algo::decodeI32(p + 64) * 1e-5;
  sAcc = algo::decodeU32(p + 68);
  headAcc = algo::decodeU32(p + 72) * 1e-5;
  pDOP = algo::decodeU16(p + 76) * 1e-2;

  const auto flags3 = algo::decodeU8(p + 78);
  invalidLlh = (flags3 >> 0) & 1;

  headVeh = algo::decodeI32(p + 84) * 1e-5;
  magDec = algo::decodeI16(p + 88) * 1e-2;
  magAcc = algo::decodeU16(p + 90) * 1e-2;
}

void NAV_RELPOSNED::decode(const uint8_t*)
{
  // TODO
}

void NAV_RESETODO::decode(const uint8_t*)
{
  // TODO
}

void NAV_SAT::decode(const uint8_t*)
{
  // TODO
}

void NAV_SBAS::decode(const uint8_t*)
{
  // TODO
}

void NAV_SIG::decode(const uint8_t*)
{
  // TODO
}

void NAV_SLAS::decode(const uint8_t*)
{
  // TODO
}

void NAV_STATUS::decode(const uint8_t* p)
{
  iTOW = algo::decodeU32(p + 0);
  gpsFix = static_cast<fix_type_t>(algo::decodeU8(p + 4));

  const auto flags = algo::decodeU8(p + 5);
  gpsFixOk = (flags >> 0) & 1;
  diffSoln = (flags >> 1) & 1;
  wknSet = (flags >> 2) & 1;
  towSet = (flags >> 3) & 1;

  const auto fixStat = algo::decodeU8(p + 6);
  diffCorr = (fixStat >> 0) & 1;
  mapMatching = static_cast<payload::NAV_STATUS::map_matching_t>((fixStat >> 6) & 0b11);

  const auto flags2 = algo::decodeU8(p + 7);
  psmState = static_cast<payload::NAV_STATUS::psm_state_t>((flags2 >> 0) & 0b11);
  spoofDetState = static_cast<payload::NAV_STATUS::spoof_det_state>((flags2 >> 3) & 0b11);

  ttff = algo::decodeU32(p + 8);
  msss = algo::decodeU32(p + 12);
}

void NAV_SVIN::decode(const uint8_t*)
{
  // TODO
}

void NAV_TIMEBDS::decode(const uint8_t*)
{
  // TODO
}

void NAV_TIMEGAL::decode(const uint8_t*)
{
  // TODO
}

void NAV_TIMEGLO::decode(const uint8_t*)
{
  // TODO
}

void NAV_TIMEGPS::decode(const uint8_t* p)
{
  iTOW = algo::decodeU32(p + 0);
  fTOW = algo::decodeI32(p + 4);
  week = algo::decodeI16(p + 8);
  leapS = algo::decodeI8(p + 10);

  const auto valid = algo::decodeU8(p + 11);
  towValid = (valid >> 0) & 1;
  weekValid = (valid >> 1) & 1;
  leapSValid = (valid >> 2) & 1;

  tAcc = algo::decodeU32(p + 12);
}

void NAV_TIMELS::decode(const uint8_t*)
{
  // TODO
}

void NAV_TIMEQZSS::decode(const uint8_t*)
{
  // TODO
}

void NAV_TIMEUTC::decode(const uint8_t*)
{
  // TODO
}

void NAV_VELECEF::decode(const uint8_t*)
{
  // TODO
}

void NAV_VELNED::decode(const uint8_t* p)
{
  iTOW = algo::decodeU32(p + 0);

  velN = algo::decodeI32(p + 4);
  velE = algo::decodeI32(p + 8);
  velD = algo::decodeI32(p + 12);

  speed = algo::decodeU32(p + 16);
  gSpeed = algo::decodeU32(p + 20);
  heading = algo::decodeI32(p + 24) * 1e-5;

  sAcc = algo::decodeU32(p + 28);
  cAcc = algo::decodeU32(p + 32) * 1e-5;
}

/* ===== Print Methods ===== */

void ACK_NAK::print(ostream& os) const
{
  os << "Class ID of the Not-Acknowledged Message: " << (int)clsID << endl;
  os << "Message ID of the Not-Acknowledged Message: " << (int)msgID << endl;
}

void ACK_ACK::print(ostream& os) const
{
  os << "Class ID of the Acknowledged Message: " << (int)clsID << endl;
  os << "Message ID of the Acknowledged Message: " << (int)msgID << endl;
}

void NAV_CLOCK::print(ostream&) const
{
  // TODO
}

void NAV_COV::print(ostream& os) const
{
  os << "GPS time of week: " << iTOW << "[ms]" << endl;
  os << "Message version (0x00 for this version): " << version << endl;
  os << "Position covariance matrix validity flag: " << posCovValid << endl;
  os << "Velocity covariance matrix validity flag: " << velCovValid << endl;

  os << "Position covariance matrix value p_NN: " << posCovNN << "[m^2]" << endl;
  os << "Position covariance matrix value p_NE: " << posCovNE << "[m^2]" << endl;
  os << "Position covariance matrix value p_ND: " << posCovND << "[m^2]" << endl;
  os << "Position covariance matrix value p_EE: " << posCovEE << "[m^2]" << endl;
  os << "Position covariance matrix value p_ED: " << posCovED << "[m^2]" << endl;
  os << "Position covariance matrix value p_DD: " << posCovDD << "[m^2]" << endl;
  os << "Velocity covariance matrix value v_NN: " << velCovNN << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_NE: " << velCovNE << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_ND: " << velCovND << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_EE: " << velCovEE << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_ED: " << velCovED << "[m^2/s^2]" << endl;
  os << "Velocity covariance matrix value v_DD: " << velCovDD << "[m^2/s^2]" << endl;
}

void NAV_DOP::print(ostream&) const
{
  // TODO
}

void NAV_EOE::print(ostream&) const
{
  // TODO
}

void NAV_GEOFENCE::print(ostream&) const
{
  // TODO
}

void NAV_HPPOSECEF::print(ostream&) const
{
  // TODO
}

void NAV_HPPOSLLH::print(ostream& os) const
{
  os << "Message version (0x00 for this version): " << version << endl;

  os << "Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp: " << invalidLlh << endl;

  os << "GPS time of week: " << iTOW << "[ms]" << endl;

  os << "Longitude: " << lon << "[deg]" << endl;
  os << "Latitude: " << lat << "[deg]" << endl;
  os << "Height above ellipsoid: " << height << "[mm]" << endl;
  os << "Height above mean sea level: " << hMSL << "[mm]" << endl;

  os << "High precision component of longitude: " << lonHp << "[deg]" << endl;
  os << "High precision component of latitude: " << latHp << "[deg]" << endl;
  os << "High precision component of height above ellipsoid: " << heightHp << "[mm]" << endl;
  os << "High precision component of height above mean sea level: " << hMSLHp << "[mm]" << endl;

  os << "Horizontal accuracy estimate: " << hAcc << "[mm]" << endl;
  os << "Vertical accuracy estimate: " << vAcc << "[mm]" << endl;
}

void NAV_ODO::print(ostream&) const
{
  // TODO
}

void NAV_ORB::print(ostream&) const
{
  // TODO
}

void NAV_PL::print(ostream&) const
{
  // TODO
}

void NAV_POSECEF::print(ostream&) const
{
  // TODO
}

void NAV_POSLLH::print(ostream& os) const
{
  os << "GPS time of week: " << iTOW << "[ms]" << endl;

  os << "Longitude: " << lon << "[deg]" << endl;
  os << "Latitude: " << lat << "[deg]" << endl;
  os << "Height above ellipsoid: " << height << "[mm]" << endl;
  os << "Height above mean sea level: " << hMSL << "[mm]" << endl;

  os << "Horizontal accuracy estimate: " << hAcc << "[mm]" << endl;
  os << "Vertical accuracy estimate: " << vAcc << "[mm]" << endl;
}

void NAV_PVT::print(ostream& os) const
{
  os << "GPS time of week: " << iTOW << "[ms]" << endl;

  os << "Year (UTC): " << (int)year << endl;
  os << "Month, range 1..12 (UTC): " << (int)month << endl;
  os << "Day of month, range 1..31 (UTC): " << (int)day << endl;
  os << "Hour of day, range 0..23 (UTC): " << (int)hour << endl;
  os << "Minute of hour, range 0..59 (UTC): " << (int)min << endl;
  os << "Seconds of minute, range 0..60 (UTC): " << (int)sec << endl;

  os << "Valid UTC Date: " << validDate << endl;
  os << "Valid UTC time of day: " << validTime << endl;
  os << "UTC time of day has been fully resolved: " << fullyResolved << endl;
  os << "Valid magnetic declination: " << validMag << endl;

  os << "Time accuracy estimate (UTC): " << tAcc << "[ns]" << endl;
  os << "Fraction of second, range -1e9 .. 1e9 (UTC): " << nano << "[ns]" << endl;

  os << "GNSSfix Type: " << (int)fixType << endl;

  os << "Valid fix (i.e within DOP & accuracy masks): " << gnssFixOk << endl;
  os << "Differential corrections were applied: " << diffSoln << endl;
  os << "Power Save Mode state: " << (int)psmState << endl;
  os << "Heading of vehicle is valid: " << headVehValid << endl;
  os << "Carrier phase range solution status: " << carrSoln << endl;

  os << "Information about UTC Date and Time of Day validity confirmation is available: " << confirmedAvai << endl;
  os << "UTC Date validity could be confirmed: " << confirmedDate << endl;
  os << "UTC Time of Day could be confirmed: " << confirmedTime << endl;

  os << "Longitude: " << lon << "[deg]" << endl;
  os << "Latitude: " << lat << "[deg]" << endl;
  os << "Height above ellipsoid: " << height << "[mm]" << endl;
  os << "Height above mean sea level: " << hMSL << "[mm]" << endl;
  os << "Horizontal accuracy estimate: " << hAcc << "[mm]" << endl;
  os << "Vertical accuracy estimate: " << vAcc << "[mm]" << endl;
  os << "NED north velocity: " << velN << "[m/s]" << endl;
  os << "NED east velocity: " << velE << "[m/s]" << endl;
  os << "NED down velocity: " << velD << "[m/s]" << endl;
  os << "Ground Speed (2-D): " << gSpeed << "[mm/s]" << endl;
  os << "Heading of motion (2-D): " << headMot << "[deg]" << endl;
  os << "Speed accuracy estimate: " << sAcc << "[mm/s]" << endl;
  os << "Heading accuracy estimate: " << headAcc << "[deg]" << endl;
  os << "Position DOP: " << pDOP << endl;

  os << "Invalid lon, lat, height and hMSL: " << invalidLlh << endl;

  os << "Heading of vehicle (2-D): " << headVeh << "[deg]" << endl;
  os << "Magnetic declination: " << magDec << "[deg]" << endl;
  os << "Magnetic declination accuracy: " << magAcc << "[deg]" << endl;
}

void NAV_RELPOSNED::print(ostream&) const
{
  // TODO
}

void NAV_RESETODO::print(ostream&) const
{
  // TODO
}

void NAV_SAT::print(ostream&) const
{
  // TODO
}

void NAV_SBAS::print(ostream&) const
{
  // TODO
}

void NAV_SIG::print(ostream&) const
{
  // TODO
}

void NAV_SLAS::print(ostream&) const
{
  // TODO
}

void NAV_STATUS::print(ostream& os) const
{
  os << "GPS time of week: " << iTOW << "[ms]" << endl;
  os << "GPSfix Type: " << (int)gpsFix << endl;

  os << "Position and velocity valid and within DOP and ACC Masks: " << gpsFixOk << endl;
  os << "Differential corrections were applied: " << diffSoln << endl;
  os << "Week Number valid: " << wknSet << endl;
  os << "Time of Week valid: " << towSet << endl;

  os << "Differential corrections available: " << diffCorr << endl;
  os << "Map matching status: " << (int)mapMatching << endl;

  os << "Power save mode state: " << (int)psmState << endl;
  os << "Spoofing detection state: " << (int)spoofDetState << endl;

  os << "Time to first fix: " << ttff << "[ms]" << endl;
  os << "Milliseconds since Startup / Reset: " << msss << "[ms]" << endl;
}

void NAV_SVIN::print(ostream&) const
{
  // TODO
}

void NAV_TIMEBDS::print(ostream&) const
{
  // TODO
}

void NAV_TIMEGAL::print(ostream&) const
{
  // TODO
}

void NAV_TIMEGLO::print(ostream&) const
{
  // TODO
}

void NAV_TIMEGPS::print(ostream& os) const
{
  os << "GPS time of week of the navigation epoch: " << iTOW << "[ms]" << endl;
  os << "Fractional part of iTOW: " << fTOW << "[ns]" << endl;
  os << "GPS week number of the navigation epoch: " << week << endl;
  os << "GPS leap seconds (GPS-UTC): " << (int)leapS << "[s]" << endl;

  os << "Valid GPS time of week: " << towValid << endl;
  os << "Valid GPS week number: " << weekValid << endl;
  os << "Valid GPS leap seconds: " << leapSValid << endl;

  os << "Time Accuracy Estimate: " << tAcc << "[ns]" << endl;
}

void NAV_TIMELS::print(ostream&) const
{
  // TODO
}

void NAV_TIMEQZSS::print(ostream&) const
{
  // TODO
}

void NAV_TIMEUTC::print(ostream&) const
{
  // TODO
}

void NAV_VELECEF::print(ostream&) const
{
  // TODO
}

void NAV_VELNED::print(ostream& os) const
{
  os << "GPS time of week: " << iTOW << "[ms]" << endl;

  os << "North velocity component: " << velN << "[cm/s]" << endl;
  os << "East velocity component: " << velE << "[cm/s]" << endl;
  os << "Down velocity component: " << velD << "[cm/s]" << endl;

  os << "Speed (3-D): " << speed << "[cm/s]" << endl;
  os << "Ground Speed (3-D): " << gSpeed << "[cm/s]" << endl;
  os << "Heading of motion 2-D: " << heading << "[deg]" << endl;

  os << "Speed accuracy estimate: " << sAcc << "[mm/s]" << endl;
  os << "Course / Heading accuracy estimate: " << cAcc << "[deg]" << endl;
}
}  // namespace payload
}  // namespace a1
