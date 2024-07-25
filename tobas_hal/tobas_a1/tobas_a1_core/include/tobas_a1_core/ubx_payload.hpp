#pragma once

#include <cinttypes>
#include <iostream>

namespace a1
{
namespace payload
{
struct Payload
{
  virtual void decode(const uint8_t* p) = 0;
  virtual void print(std::ostream& os) const = 0;

  friend std::ostream& operator<<(std::ostream& os, const Payload& arg)
  {
    arg.print(os);
    return os;
  }
};

struct ACK_NAK : public Payload
{
  uint8_t clsID;  // Class ID of the Not-Acknowledged Message
  uint8_t msgID;  // Message ID of the Not-Acknowledged Message

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct ACK_ACK : public Payload
{
  uint8_t clsID;  // Class ID of the Acknowledged Message
  uint8_t msgID;  // Message ID of the Acknowledged Message

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_CLOCK : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_COV : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_DOP : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_EOE : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_GEOFENCE : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_HPPOSECEF : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_HPPOSLLH : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_ODO : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_ORB : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_PL : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_POSECEF : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_POSLLH : public Payload
{
  uint32_t iTOW;  // GPS time of week [ms]

  double lon;      // Longitude [deg]
  double lat;      // Latitude [deg]
  int32_t height;  // Height above ellipsoid [mm]
  int32_t hMSL;    // Height above mean sea level [mm]

  uint32_t hAcc;  // Horizontal accuracy estimate [mm]
  uint32_t vAcc;  // Vertical accuracy estimate [mm]

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_PVT : public Payload
{
  uint32_t iTOW;  // GPS time of week [ms]

  uint16_t year;  // Year (UTC) [y]
  uint8_t month;  // Month, range 1..12 (UTC) [month]
  uint8_t day;    // Day of month, range 1..31 (UTC) [d]
  uint8_t hour;   // Hour of day, range 0..23 (UTC) [h]
  uint8_t min;    // Minute of hour, range 0..59 (UTC) [min]
  uint8_t sec;    // Seconds of minute, range 0..60 (UTC) [s]

  // valid: Validity flags
  bool validDate;      // Valid UTC Date
  bool validTime;      // Valid UTC Time of Day
  bool fullyResolved;  // UTC Time of Day has been fully resolved
  bool validMag;       // Valid Magnetic declination

  uint32_t tAcc;  // Time accuracy estimate (UTC) [ns]
  int32_t nano;   // Fraction of second, range -1e9 .. 1e9 (UTC) [ns]

  // GNSSfix Type
  enum fix_type_t : uint8_t
  {
    NO_FIX = 0,
    DEAD_RECKONING_ONLY = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    GNSS_DEAD_RECKONING_COMBINED = 4,
    TIME_ONLY_FIX = 5,
  } fixType;

  // flags: Fix status flags
  bool gnssFixOk;  // Valid fix (i.e within DOP & accuracy masks)
  bool diffSoln;   // Differential corrections were applied
  // Power Save Mode state
  enum psm_state_t : uint8_t
  {
    NOT_ACTIVE = 0,
    ENABLED = 1,
    ACQUISITION = 2,
    TRACKING = 3,
    POWER_OPTIMIXED_TRACKING = 4,
    INACTIVE = 4,
  } psmState;
  bool headVehValid;  // Heading of vehicle is valid
                      // Carrier phase range solution status
  enum carr_soln_t : uint8_t
  {
    NONE = 0,      // No carrier phase range solution
    FLOATING = 1,  // Carrier phase range solution with floating ambiguities
    FIXED = 2,     // Carrier phase range solution with fixed ambiguities
  } carrSoln;

  // flags2: Additional flags
  bool confirmedAvai;  // Information about UTC Date and Time of Day validity confirmation is available
  bool confirmedDate;  // UTC Date validity could be confirmed
  bool confirmedTime;  // UTC Time of Day could be confirmed

  uint8_t numSV;  // Number of satellites used in Nav Solution

  double lon;      // Longitude [deg]
  double lat;      // Latitude [deg]
  int32_t height;  // Height above ellipsoid [mm]
  int32_t hMSL;    // Height above mean sea level [mm]
  uint32_t hAcc;   // Horizontal accuracy estimate [mm]
  uint32_t vAcc;   // Vertical accuracy estimate [mm]
  int32_t velN;    // NED north veloity [mm/s]
  int32_t velE;    // NED east veloity [mm/s]
  int32_t velD;    // NED down veloity [mm/s]
  int32_t gSpeed;  // Ground Speed (2-D) [mm/s]
  double headMot;  // Heading of motion (2-D) [deg]
  uint32_t sAcc;   // Speed accuracy estimate [mm/s]
  double headAcc;  // Heading accuracy estimate [deg]
  double pDOP;     // Position DOP

  // flags3: Additional flags
  bool invalidLlh;  // Invalid lon, lat, height and hMSL

  double headVeh;  // Heading of vehicle (2-D) [deg]
  double magDec;   // Magnetic declination [deg]
  double magAcc;   // Magnetic declination accuracy [deg]

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_RELPOSNED : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_RESETODO : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_SAT : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_SBAS : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_SIG : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_SLAS : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_STATUS : public Payload
{
  uint32_t iTOW;   // GPS time of week [ms]
  uint8_t gpsFix;  // GPSfix Type, this value does not qualify a fix as valid and within the limits

  // flags: Navigation Status Flags
  bool gpsFixOk;  // Position and velocity valid and within DOP and ACC Masks
  bool diffSoln;  // Differential corrections were applied
  bool wknSet;    // Week Number valid
  bool towSet;    // Time of Week valid

  // fixStat: Fix Status Information
  bool diffCorr;  // Differential corrections available
  // Map matching status
  enum map_matching_t : uint8_t
  {
    // None
    NONE = 0b00,
    // Valid but not used, i.e. map matching data was received, but was too old
    VALID_BUT_NOT_USED = 0b01,
    // Valid and used, map matching data was applied
    VALID_AND_USED_1 = 0b10,
    // Valid and used, map matching data was applied.
    // In case of sensor unavailability map matching data enables dead reckoning.
    // This requires map matched latitude/longitude or heading data.
    VALID_AND_USED_2 = 0b11,
  } mapMatching;

  // flags2: Further information about navigation output
  // Power save mode state
  enum psm_state_t : uint8_t
  {
    ACQUISITION = 0,
    TRACKING = 1,
    POWER_OPTIMIXED_TRACKING = 2,
    INACTIVE = 3,
  } psmState;
  // Spoofing detection state
  enum spoof_det_state : uint8_t
  {
    UNKNOWN_OR_DEACTIVATED = 0,
    NO_SPOOFING_INDICATED = 1,
    SPOOFING_INDICATED = 2,
    MULTIPLE_SPOOFING_INDICATIONS = 3,
  } spoofDetState;

  uint32_t ttff;  // Time to first fix [ms]
  uint32_t msss;  // Milliseconds since Startup / Reset [ms]

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_SVIN : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_TIMEBDS : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_TIMEGAL : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_TIMEGLO : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_TIMEGPS : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_TIMELS : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_TIMEQZSS : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_TIMEUTC : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_VELECEF : public Payload
{
  // TODO

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_VELNED : public Payload
{
  uint32_t iTOW;  // GPS time of week [ms]

  int32_t velN;  // North velocity component [cm/s]
  int32_t velE;  // East velocity component [cm/s]
  int32_t velD;  // Down velocity component [cm/s]

  uint32_t speed;   // Speed (3-D) [cm/s]
  uint32_t gSpeed;  // Ground Speed (2-D) [cm/s]
  double heading;   // Heading of motion 2-D [deg]

  uint32_t sAcc;  // Speed accuracy estimate [cm/s]
  double cAcc;    // Course / Heading accuracy estimate [deg]

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};
}  // namespace payload
}  // namespace a1
