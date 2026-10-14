// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_payload.hpp"

#include <bit>
#include <ranges>

#include <tobas_algorithm/binary.hpp>

namespace tobas
{
namespace ublox
{
namespace payload
{
/* ===== Decode Methods ===== */

void ACK_NAK::decode(const uint8_t* _p)
{
  clsID = algo::decodeU8(_p + 0);
  msgID = algo::decodeU8(_p + 1);
}

void ACK_ACK::decode(const uint8_t* _p)
{
  clsID = algo::decodeU8(_p + 0);
  msgID = algo::decodeU8(_p + 1);
}

void NAV_CLOCK::decode(const uint8_t*)
{
  // TODO
}

void NAV_COV::decode(const uint8_t* _p)
{
  iTOW = algo::decodeU32(_p + 0);
  version = algo::decodeU8(_p + 4);
  posCovValid = algo::decodeU8(_p + 5);
  velCovValid = algo::decodeU8(_p + 6);

  posCovNN = std::bit_cast<float>(algo::decodeU32(_p + 16));
  posCovNE = std::bit_cast<float>(algo::decodeU32(_p + 20));
  posCovND = std::bit_cast<float>(algo::decodeU32(_p + 24));
  posCovEE = std::bit_cast<float>(algo::decodeU32(_p + 28));
  posCovED = std::bit_cast<float>(algo::decodeU32(_p + 32));
  posCovDD = std::bit_cast<float>(algo::decodeU32(_p + 36));
  velCovNN = std::bit_cast<float>(algo::decodeU32(_p + 40));
  velCovNE = std::bit_cast<float>(algo::decodeU32(_p + 44));
  velCovND = std::bit_cast<float>(algo::decodeU32(_p + 48));
  velCovEE = std::bit_cast<float>(algo::decodeU32(_p + 52));
  velCovED = std::bit_cast<float>(algo::decodeU32(_p + 56));
  velCovDD = std::bit_cast<float>(algo::decodeU32(_p + 60));
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

void NAV_HPPOSLLH::decode(const uint8_t* _p)
{
  version = algo::decodeU8(_p + 0);

  const auto flags = algo::decodeU8(_p + 3);
  invalidLlh = (flags >> 0) & 1;

  iTOW = algo::decodeU32(_p + 4);

  lon = algo::decodeI32(_p + 8) * 1e-7;
  lat = algo::decodeI32(_p + 12) * 1e-7;
  height = algo::decodeI32(_p + 16);
  hMSL = algo::decodeI32(_p + 20);

  lonHp = algo::decodeI8(_p + 24) * 1e-9;
  latHp = algo::decodeI8(_p + 25) * 1e-9;
  heightHp = algo::decodeI8(_p + 26) * 0.1;
  hMSLHp = algo::decodeI8(_p + 27) * 0.1;

  hAcc = algo::decodeU32(_p + 28) * 0.1;
  vAcc = algo::decodeU32(_p + 32) * 0.1;
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

void NAV_POSLLH::decode(const uint8_t* _p)
{
  iTOW = algo::decodeU32(_p + 0);

  lon = algo::decodeI32(_p + 4) * 1e-7;
  lat = algo::decodeI32(_p + 8) * 1e-7;
  height = algo::decodeI32(_p + 12);
  hMSL = algo::decodeI32(_p + 16);

  hAcc = algo::decodeU32(_p + 20);
  vAcc = algo::decodeU32(_p + 24);
}

void NAV_PVT::decode(const uint8_t* _p)
{
  iTOW = algo::decodeU32(_p + 0);

  year = algo::decodeU16(_p + 4);
  month = algo::decodeU8(_p + 6);
  day = algo::decodeU8(_p + 7);
  hour = algo::decodeU8(_p + 8);
  min = algo::decodeU8(_p + 9);
  sec = algo::decodeU8(_p + 10);

  const auto valid = algo::decodeU8(_p + 11);
  validDate = (valid >> 0) & 1;
  validTime = (valid >> 1) & 1;
  fullyResolved = (valid >> 2) & 1;
  validMag = (valid >> 3) & 1;

  tAcc = algo::decodeU32(_p + 12);
  nano = algo::decodeI32(_p + 16);

  fixType = static_cast<FixType>(algo::decodeU8(_p + 20));

  const auto flags = algo::decodeU8(_p + 21);
  gnssFixOk = (flags >> 0) & 1;
  diffSoln = (flags >> 1) & 1;
  psmState = static_cast<payload::NAV_PVT::PowerSaveModeState>((flags >> 2) & 0b111);
  headVehValid = (flags >> 5) & 1;
  carrSoln = static_cast<payload::NAV_PVT::CarrierPhaseRangeSolution>((flags >> 6) & 0b11);

  const auto flags2 = algo::decodeU8(_p + 22);
  confirmedAvai = (flags2 >> 5) & 1;
  confirmedDate = (flags2 >> 6) & 1;
  confirmedTime = (flags2 >> 7) & 1;

  numSV = algo::decodeU8(_p + 23);

  lon = algo::decodeI32(_p + 24) * 1e-7;
  lat = algo::decodeI32(_p + 28) * 1e-7;
  height = algo::decodeI32(_p + 32);
  hMSL = algo::decodeI32(_p + 36);
  hAcc = algo::decodeU32(_p + 40);
  vAcc = algo::decodeU32(_p + 44);
  velN = algo::decodeI32(_p + 48);
  velE = algo::decodeI32(_p + 52);
  velD = algo::decodeI32(_p + 56);
  gSpeed = algo::decodeI32(_p + 60);
  headMot = algo::decodeI32(_p + 64) * 1e-5;
  sAcc = algo::decodeU32(_p + 68);
  headAcc = algo::decodeU32(_p + 72) * 1e-5;
  pDOP = algo::decodeU16(_p + 76) * 1e-2;

  const auto flags3 = algo::decodeU8(_p + 78);
  invalidLlh = (flags3 >> 0) & 1;

  headVeh = algo::decodeI32(_p + 84) * 1e-5;
  magDec = algo::decodeI16(_p + 88) * 1e-2;
  magAcc = algo::decodeU16(_p + 90) * 1e-2;
}

void NAV_RELPOSNED::decode(const uint8_t*)
{
  // TODO
}

void NAV_RESETODO::decode(const uint8_t*)
{
  // TODO
}

void NAV_SAT::decode(const uint8_t* _p)
{
  iTOW = algo::decodeU32(_p + 0);
  version = algo::decodeU8(_p + 4);
  numSvs = algo::decodeU8(_p + 5);

  satellites.resize(numSvs);

  for (size_t i = 0; i < numSvs; ++i) {
    const auto sv = _p + 8 + i * 12;
    auto& sat = satellites[i];

    sat.gnssId = algo::decodeU8(sv + 0);
    sat.svId = algo::decodeU8(sv + 1);
    sat.cno = algo::decodeU8(sv + 2);
    sat.elev = algo::decodeI8(sv + 3);
    sat.azim = algo::decodeI16(sv + 4);
    sat.prRes = algo::decodeI16(sv + 6) * 0.1;

    const auto flags = algo::decodeU32(sv + 8);
    sat.qualityInd = static_cast<NAV_SAT::Satellite::QualityIndicator>(flags & 0b111);
    sat.svUsed = (flags >> 3) & 1;
    sat.health = static_cast<NAV_SAT::Satellite::Health>((flags >> 4) & 0b11);
    sat.diffCorr = (flags >> 6) & 1;
    sat.smoothed = (flags >> 7) & 1;
    sat.orbitSource = static_cast<NAV_SAT::Satellite::OrbitSource>((flags >> 8) & 0b111);
    sat.ephAvail = (flags >> 11) & 1;
    sat.almAvail = (flags >> 12) & 1;
    sat.anoAvail = (flags >> 13) & 1;
    sat.aopAvail = (flags >> 14) & 1;
    sat.sbasCorrUsed = (flags >> 16) & 1;
    sat.rtcmCorrUsed = (flags >> 17) & 1;
    sat.slasCorrUsed = (flags >> 18) & 1;
    sat.spartnCorrUsed = (flags >> 19) & 1;
    sat.prCorrUsed = (flags >> 20) & 1;
    sat.crCorrUsed = (flags >> 21) & 1;
    sat.doCorrUsed = (flags >> 22) & 1;
    sat.clasCorrUsed = (flags >> 23) & 1;
  }
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

void NAV_STATUS::decode(const uint8_t* _p)
{
  iTOW = algo::decodeU32(_p + 0);
  gpsFix = static_cast<FixType>(algo::decodeU8(_p + 4));

  const auto flags = algo::decodeU8(_p + 5);
  gpsFixOk = (flags >> 0) & 1;
  diffSoln = (flags >> 1) & 1;
  wknSet = (flags >> 2) & 1;
  towSet = (flags >> 3) & 1;

  const auto fixStat = algo::decodeU8(_p + 6);
  diffCorr = (fixStat >> 0) & 1;
  mapMatching = static_cast<payload::NAV_STATUS::MapMatchingStatus>((fixStat >> 6) & 0b11);

  const auto flags2 = algo::decodeU8(_p + 7);
  psmState = static_cast<payload::NAV_STATUS::PowerSaveModeState>((flags2 >> 0) & 0b11);
  spoofDetState = static_cast<payload::NAV_STATUS::SpoofDetectionState>((flags2 >> 3) & 0b11);

  ttff = algo::decodeU32(_p + 8);
  msss = algo::decodeU32(_p + 12);
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

void NAV_TIMEGPS::decode(const uint8_t* _p)
{
  iTOW = algo::decodeU32(_p + 0);
  fTOW = algo::decodeI32(_p + 4);
  week = algo::decodeI16(_p + 8);
  leapS = algo::decodeI8(_p + 10);

  const auto valid = algo::decodeU8(_p + 11);
  towValid = (valid >> 0) & 1;
  weekValid = (valid >> 1) & 1;
  leapSValid = (valid >> 2) & 1;

  tAcc = algo::decodeU32(_p + 12);
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

void NAV_VELNED::decode(const uint8_t* _p)
{
  iTOW = algo::decodeU32(_p + 0);

  velN = algo::decodeI32(_p + 4);
  velE = algo::decodeI32(_p + 8);
  velD = algo::decodeI32(_p + 12);

  speed = algo::decodeU32(_p + 16);
  gSpeed = algo::decodeU32(_p + 20);
  heading = algo::decodeI32(_p + 24) * 1e-5;

  sAcc = algo::decodeU32(_p + 28);
  cAcc = algo::decodeU32(_p + 32) * 1e-5;
}

/* ===== Print Methods ===== */

void ACK_NAK::print(std::ostream& _os) const
{
  _os << "Class ID of the Not-Acknowledged Message: " << (int)clsID << std::endl;
  _os << "Message ID of the Not-Acknowledged Message: " << (int)msgID << std::endl;
}

void ACK_ACK::print(std::ostream& _os) const
{
  _os << "Class ID of the Acknowledged Message: " << (int)clsID << std::endl;
  _os << "Message ID of the Acknowledged Message: " << (int)msgID << std::endl;
}

void NAV_CLOCK::print(std::ostream&) const
{
  // TODO
}

void NAV_COV::print(std::ostream& _os) const
{
  _os << "GPS time of week: " << iTOW << "[ms]" << std::endl;
  _os << "Message version (0x00 for this version): " << version << std::endl;
  _os << "Position covariance matrix validity flag: " << posCovValid << std::endl;
  _os << "Velocity covariance matrix validity flag: " << velCovValid << std::endl;

  _os << "Position covariance matrix value p_NN: " << posCovNN << "[m^2]" << std::endl;
  _os << "Position covariance matrix value p_NE: " << posCovNE << "[m^2]" << std::endl;
  _os << "Position covariance matrix value p_ND: " << posCovND << "[m^2]" << std::endl;
  _os << "Position covariance matrix value p_EE: " << posCovEE << "[m^2]" << std::endl;
  _os << "Position covariance matrix value p_ED: " << posCovED << "[m^2]" << std::endl;
  _os << "Position covariance matrix value p_DD: " << posCovDD << "[m^2]" << std::endl;
  _os << "Velocity covariance matrix value v_NN: " << velCovNN << "[m^2/s^2]" << std::endl;
  _os << "Velocity covariance matrix value v_NE: " << velCovNE << "[m^2/s^2]" << std::endl;
  _os << "Velocity covariance matrix value v_ND: " << velCovND << "[m^2/s^2]" << std::endl;
  _os << "Velocity covariance matrix value v_EE: " << velCovEE << "[m^2/s^2]" << std::endl;
  _os << "Velocity covariance matrix value v_ED: " << velCovED << "[m^2/s^2]" << std::endl;
  _os << "Velocity covariance matrix value v_DD: " << velCovDD << "[m^2/s^2]" << std::endl;
}

void NAV_DOP::print(std::ostream&) const
{
  // TODO
}

void NAV_EOE::print(std::ostream&) const
{
  // TODO
}

void NAV_GEOFENCE::print(std::ostream&) const
{
  // TODO
}

void NAV_HPPOSECEF::print(std::ostream&) const
{
  // TODO
}

void NAV_HPPOSLLH::print(std::ostream& _os) const
{
  _os << "Message version (0x00 for this version): " << version << std::endl;

  _os << "Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp: " << invalidLlh << std::endl;

  _os << "GPS time of week: " << iTOW << "[ms]" << std::endl;

  _os << "Longitude: " << lon << "[deg]" << std::endl;
  _os << "Latitude: " << lat << "[deg]" << std::endl;
  _os << "Height above ellipsoid: " << height << "[mm]" << std::endl;
  _os << "Height above mean sea level: " << hMSL << "[mm]" << std::endl;

  _os << "High precision component of longitude: " << lonHp << "[deg]" << std::endl;
  _os << "High precision component of latitude: " << latHp << "[deg]" << std::endl;
  _os << "High precision component of height above ellipsoid: " << heightHp << "[mm]" << std::endl;
  _os << "High precision component of height above mean sea level: " << hMSLHp << "[mm]" << std::endl;

  _os << "Horizontal accuracy estimate: " << hAcc << "[mm]" << std::endl;
  _os << "Vertical accuracy estimate: " << vAcc << "[mm]" << std::endl;
}

void NAV_ODO::print(std::ostream&) const
{
  // TODO
}

void NAV_ORB::print(std::ostream&) const
{
  // TODO
}

void NAV_PL::print(std::ostream&) const
{
  // TODO
}

void NAV_POSECEF::print(std::ostream&) const
{
  // TODO
}

void NAV_POSLLH::print(std::ostream& _os) const
{
  _os << "GPS time of week: " << iTOW << "[ms]" << std::endl;

  _os << "Longitude: " << lon << "[deg]" << std::endl;
  _os << "Latitude: " << lat << "[deg]" << std::endl;
  _os << "Height above ellipsoid: " << height << "[mm]" << std::endl;
  _os << "Height above mean sea level: " << hMSL << "[mm]" << std::endl;

  _os << "Horizontal accuracy estimate: " << hAcc << "[mm]" << std::endl;
  _os << "Vertical accuracy estimate: " << vAcc << "[mm]" << std::endl;
}

void NAV_PVT::print(std::ostream& _os) const
{
  _os << "GPS time of week: " << iTOW << "[ms]" << std::endl;

  _os << "Year (UTC): " << (int)year << std::endl;
  _os << "Month, range 1..12 (UTC): " << (int)month << std::endl;
  _os << "Day of month, range 1..31 (UTC): " << (int)day << std::endl;
  _os << "Hour of day, range 0..23 (UTC): " << (int)hour << std::endl;
  _os << "Minute of hour, range 0..59 (UTC): " << (int)min << std::endl;
  _os << "Seconds of minute, range 0..60 (UTC): " << (int)sec << std::endl;

  _os << "Valid UTC Date: " << validDate << std::endl;
  _os << "Valid UTC time of day: " << validTime << std::endl;
  _os << "UTC time of day has been fully resolved: " << fullyResolved << std::endl;
  _os << "Valid magnetic declination: " << validMag << std::endl;

  _os << "Time accuracy estimate (UTC): " << tAcc << "[ns]" << std::endl;
  _os << "Fraction of second, range -1e9 .. 1e9 (UTC): " << nano << "[ns]" << std::endl;

  _os << "GNSSfix Type: " << (int)fixType << std::endl;

  _os << "Valid fix (i.e within DOP & accuracy masks): " << gnssFixOk << std::endl;
  _os << "Differential corrections were applied: " << diffSoln << std::endl;
  _os << "Power Save Mode state: " << (int)psmState << std::endl;
  _os << "Heading of vehicle is valid: " << headVehValid << std::endl;
  _os << "Carrier phase range solution status: " << carrSoln << std::endl;

  _os << "Information about UTC Date and Time of Day validity confirmation is available: " << confirmedAvai
      << std::endl;
  _os << "UTC Date validity could be confirmed: " << confirmedDate << std::endl;
  _os << "UTC Time of Day could be confirmed: " << confirmedTime << std::endl;

  _os << "Number of satellites used in Nav Solution: " << (int)numSV << std::endl;

  _os << "Longitude: " << lon << "[deg]" << std::endl;
  _os << "Latitude: " << lat << "[deg]" << std::endl;
  _os << "Height above ellipsoid: " << height << "[mm]" << std::endl;
  _os << "Height above mean sea level: " << hMSL << "[mm]" << std::endl;
  _os << "Horizontal accuracy estimate: " << hAcc << "[mm]" << std::endl;
  _os << "Vertical accuracy estimate: " << vAcc << "[mm]" << std::endl;
  _os << "NED north velocity: " << velN << "[m/s]" << std::endl;
  _os << "NED east velocity: " << velE << "[m/s]" << std::endl;
  _os << "NED down velocity: " << velD << "[m/s]" << std::endl;
  _os << "Ground Speed (2-D): " << gSpeed << "[mm/s]" << std::endl;
  _os << "Heading of motion (2-D): " << headMot << "[deg]" << std::endl;
  _os << "Speed accuracy estimate: " << sAcc << "[mm/s]" << std::endl;
  _os << "Heading accuracy estimate: " << headAcc << "[deg]" << std::endl;
  _os << "Position DOP: " << pDOP << std::endl;

  _os << "Invalid lon, lat, height and hMSL: " << invalidLlh << std::endl;

  _os << "Heading of vehicle (2-D): " << headVeh << "[deg]" << std::endl;
  _os << "Magnetic declination: " << magDec << "[deg]" << std::endl;
  _os << "Magnetic declination accuracy: " << magAcc << "[deg]" << std::endl;
}

void NAV_RELPOSNED::print(std::ostream&) const
{
  // TODO
}

void NAV_RESETODO::print(std::ostream&) const
{
  // TODO
}

void NAV_SAT::print(std::ostream& _os) const
{
  _os << "GPS time of week: " << iTOW << "[ms]" << std::endl;
  _os << "Message version: " << (int)version << std::endl;
  _os << "Number of satellites: " << (int)numSvs << std::endl;

  for (const auto& [i, sat] : std::views::enumerate(satellites)) {
    _os << "Satellite[" << i << "] " << "gnssId=" << (int)sat.gnssId << ", " << "svId=" << (int)sat.svId << ", "
        << "cno=" << (int)sat.cno << "[dB-Hz], " << "elev=" << (int)sat.elev << "[deg], " << "azim=" << sat.azim
        << "[deg], " << "prRes=" << sat.prRes << "[m], " << "qualityInd=" << (int)sat.qualityInd << ", "
        << "svUsed=" << sat.svUsed << ", " << "health=" << (int)sat.health << ", " << "diffCorr=" << sat.diffCorr
        << ", " << "smoothed=" << sat.smoothed << ", " << "orbitSource=" << (int)sat.orbitSource << ", "
        << "ephAvail=" << sat.ephAvail << ", " << "almAvail=" << sat.almAvail << ", " << "anoAvail=" << sat.anoAvail
        << ", " << "aopAvail=" << sat.aopAvail << ", " << "sbasCorrUsed=" << sat.sbasCorrUsed << ", "
        << "rtcmCorrUsed=" << sat.rtcmCorrUsed << ", " << "slasCorrUsed=" << sat.slasCorrUsed << ", "
        << "spartnCorrUsed=" << sat.spartnCorrUsed << ", " << "prCorrUsed=" << sat.prCorrUsed << ", "
        << "crCorrUsed=" << sat.crCorrUsed << ", " << "doCorrUsed=" << sat.doCorrUsed << ", "
        << "clasCorrUsed=" << sat.clasCorrUsed << std::endl;
  }
}

void NAV_SBAS::print(std::ostream&) const
{
  // TODO
}

void NAV_SIG::print(std::ostream&) const
{
  // TODO
}

void NAV_SLAS::print(std::ostream&) const
{
  // TODO
}

void NAV_STATUS::print(std::ostream& _os) const
{
  _os << "GPS time of week: " << iTOW << "[ms]" << std::endl;
  _os << "GPSfix Type: " << (int)gpsFix << std::endl;

  _os << "Position and velocity valid and within DOP and ACC Masks: " << gpsFixOk << std::endl;
  _os << "Differential corrections were applied: " << diffSoln << std::endl;
  _os << "Week Number valid: " << wknSet << std::endl;
  _os << "Time of Week valid: " << towSet << std::endl;

  _os << "Differential corrections available: " << diffCorr << std::endl;
  _os << "Map matching status: " << (int)mapMatching << std::endl;

  _os << "Power save mode state: " << (int)psmState << std::endl;
  _os << "Spoofing detection state: " << (int)spoofDetState << std::endl;

  _os << "Time to first fix: " << ttff << "[ms]" << std::endl;
  _os << "Milliseconds since Startup / Reset: " << msss << "[ms]" << std::endl;
}

void NAV_SVIN::print(std::ostream&) const
{
  // TODO
}

void NAV_TIMEBDS::print(std::ostream&) const
{
  // TODO
}

void NAV_TIMEGAL::print(std::ostream&) const
{
  // TODO
}

void NAV_TIMEGLO::print(std::ostream&) const
{
  // TODO
}

void NAV_TIMEGPS::print(std::ostream& _os) const
{
  _os << "GPS time of week of the navigation epoch: " << iTOW << "[ms]" << std::endl;
  _os << "Fractional part of iTOW: " << fTOW << "[ns]" << std::endl;
  _os << "GPS week number of the navigation epoch: " << week << std::endl;
  _os << "GPS leap seconds (GPS-UTC): " << (int)leapS << "[s]" << std::endl;

  _os << "Valid GPS time of week: " << towValid << std::endl;
  _os << "Valid GPS week number: " << weekValid << std::endl;
  _os << "Valid GPS leap seconds: " << leapSValid << std::endl;

  _os << "Time Accuracy Estimate: " << tAcc << "[ns]" << std::endl;
}

void NAV_TIMELS::print(std::ostream&) const
{
  // TODO
}

void NAV_TIMEQZSS::print(std::ostream&) const
{
  // TODO
}

void NAV_TIMEUTC::print(std::ostream&) const
{
  // TODO
}

void NAV_VELECEF::print(std::ostream&) const
{
  // TODO
}

void NAV_VELNED::print(std::ostream& _os) const
{
  _os << "GPS time of week: " << iTOW << "[ms]" << std::endl;

  _os << "North velocity component: " << velN << "[cm/s]" << std::endl;
  _os << "East velocity component: " << velE << "[cm/s]" << std::endl;
  _os << "Down velocity component: " << velD << "[cm/s]" << std::endl;

  _os << "Speed (3-D): " << speed << "[cm/s]" << std::endl;
  _os << "Ground Speed (3-D): " << gSpeed << "[cm/s]" << std::endl;
  _os << "Heading of motion 2-D: " << heading << "[deg]" << std::endl;

  _os << "Speed accuracy estimate: " << sAcc << "[mm/s]" << std::endl;
  _os << "Course / Heading accuracy estimate: " << cAcc << "[deg]" << std::endl;
}
}  // namespace payload
}  // namespace ublox
}  // namespace tobas
