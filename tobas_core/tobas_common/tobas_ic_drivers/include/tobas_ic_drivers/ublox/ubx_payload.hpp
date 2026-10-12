// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

namespace tobas
{
namespace ublox
{
namespace payload
{
/** GNSSfix Type */
enum FixType : uint8_t
{
  NO_FIX = 0,
  DEAD_RECKONING_ONLY = 1,
  FIX_2D = 2,
  FIX_3D = 3,
  GPS_DEAD_RECKONING_COMBINED = 4,
  TIME_ONLY_FIX = 5,
};

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
  uint8_t clsID;  ///< Class ID of the Not-Acknowledged Message
  uint8_t msgID;  ///< Message ID of the Not-Acknowledged Message

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct ACK_ACK : public Payload
{
  uint8_t clsID;  ///< Class ID of the Acknowledged Message
  uint8_t msgID;  ///< Message ID of the Acknowledged Message

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct MON_RF : public Payload
{
  struct Block
  {
    uint8_t blockId;  // RF block ID (0 = L1 band, 1 = L2 or L5 band depending on product configuration)
    enum JammingState : uint8_t
    {
      JAMMING_UNKNOWN = 0,
      JAMMING_OK = 1,
      JAMMING_WARNING = 2,
      JAMMING_CRITICAL = 3,
    } jammingState;  // Output from jamming/interference monitor (deprecated by u-blox, use `jamInd` instead)
    enum AntennaStatus : uint8_t
    {
      ANT_INIT = 0,
      ANT_DONTKNOW = 1,
      ANT_OK = 2,
      ANT_SHORT = 3,
      ANT_OPEN = 4,
    } antStatus;  // Status of the antenna supervisor state machine
    enum AntennaPower : uint8_t
    {
      ANT_POWER_OFF = 0,
      ANT_POWER_ON = 1,
      ANT_POWER_DONTKNOW = 2,
    } antPower;           // Current power status of antenna
    uint32_t postStatus;  // POST status word
    uint16_t noisePerMS;  // Noise level as measured by the GPS core
    uint16_t agcCnt;      // AGC monitor (counts SIGHI xor SIGLO, range 0 to 8191)
    uint8_t jamInd;       // CW jamming indicator, scaled (0 = no CW jamming, 255 = strong CW jamming)
    int8_t ofsI;          // Imbalance of I-part of complex signal, scaled (-128 = max. negative imbalance)
    uint8_t magI;         // Magnitude of I-part of complex signal, scaled (0 = no signal, 255 = max. magnitude)
    int8_t ofsQ;          // Imbalance of Q-part of complex signal, scaled (-128 = max. negative imbalance)
    uint8_t magQ;         // Magnitude of Q-part of complex signal, scaled (0 = no signal, 255 = max. magnitude)
  };

  uint8_t version;  // Message version
  uint8_t nBlocks;  // Number of RF blocks included

  std::vector<Block> blocks;

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
  uint32_t iTOW;     ///< GPS time of week [ms]
  uint8_t version;   ///< Message version (0x00 for this version)
  bool posCovValid;  ///< Position covariance matrix validity flag
  bool velCovValid;  ///< Velocity covariance matrix validity flag

  float posCovNN;  ///< Position covariance matrix value p_NN [m^2]
  float posCovNE;  ///< Position covariance matrix value p_NE [m^2]
  float posCovND;  ///< Position covariance matrix value p_ND [m^2]
  float posCovEE;  ///< Position covariance matrix value p_EE [m^2]
  float posCovED;  ///< Position covariance matrix value p_ED [m^2]
  float posCovDD;  ///< Position covariance matrix value p_DD [m^2]
  float velCovNN;  ///< Velocity covariance matrix value v_NN [m^2/s^2]
  float velCovNE;  ///< Velocity covariance matrix value v_NE [m^2/s^2]
  float velCovND;  ///< Velocity covariance matrix value v_ND [m^2/s^2]
  float velCovEE;  ///< Velocity covariance matrix value v_EE [m^2/s^2]
  float velCovED;  ///< Velocity covariance matrix value v_ED [m^2/s^2]
  float velCovDD;  ///< Velocity covariance matrix value v_DD [m^2/s^2]

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
  uint8_t version;  ///< Message version (0x00 for this version)

  // flags: Additional flags
  bool invalidLlh;  ///< Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp

  uint32_t iTOW;  ///< GPS time of week [ms]

  double lon;      ///< Longitude [deg]
  double lat;      ///< Latitude [deg]
  int32_t height;  ///< Height above ellipsoid [mm]
  int32_t hMSL;    ///< Height above mean sea level [mm]

  double lonHp;     ///< High precision component of longitude [deg]
  double latHp;     ///< High precision component of latitude [deg]
  double heightHp;  ///< High precision component of height above ellipsoid [mm]
  double hMSLHp;    ///< High precision component of height above mean sea level [mm]

  uint32_t hAcc;  ///< Horizontal accuracy estimate [mm]
  uint32_t vAcc;  ///< Vertical accuracy estimate [mm]

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
  uint32_t iTOW;  ///< GPS time of week [ms]

  double lon;      ///< Longitude [deg]
  double lat;      ///< Latitude [deg]
  int32_t height;  ///< Height above ellipsoid [mm]
  int32_t hMSL;    ///< Height above mean sea level [mm]

  uint32_t hAcc;  ///< Horizontal accuracy estimate [mm]
  uint32_t vAcc;  ///< Vertical accuracy estimate [mm]

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct NAV_PVT : public Payload
{
  uint32_t iTOW;  ///< GPS time of week [ms]

  uint16_t year;  ///< Year (UTC) [y]
  uint8_t month;  ///< Month, range 1..12 (UTC) [month]
  uint8_t day;    ///< Day of month, range 1..31 (UTC) [d]
  uint8_t hour;   ///< Hour of day, range 0..23 (UTC) [h]
  uint8_t min;    ///< Minute of hour, range 0..59 (UTC) [min]
  uint8_t sec;    ///< Seconds of minute, range 0..60 (UTC) [s]

  // valid: Validity flags
  bool validDate;      ///< Valid UTC Date
  bool validTime;      ///< Valid UTC Time of Day
  bool fullyResolved;  ///< UTC Time of Day has been fully resolved
  bool validMag;       ///< Valid Magnetic declination

  uint32_t tAcc;  ///< Time accuracy estimate (UTC) [ns]
  int32_t nano;   ///< Fraction of second, range -1e9 .. 1e9 (UTC) [ns]

  FixType fixType;  ///< GNSSfix Type

  // flags: Fix status flags
  bool gnssFixOk;  ///< Valid fix (i.e within DOP & accuracy masks)
  bool diffSoln;   ///< Differential corrections were applied
  /** Power Save Mode state */
  enum PowerSaveModeState : uint8_t
  {
    NOT_ACTIVE = 0,
    ENABLED = 1,
    ACQUISITION = 2,
    TRACKING = 3,
    POWER_OPTIMIXED_TRACKING = 4,
    INACTIVE = 4,
  } psmState;
  bool headVehValid;  ///< Heading of vehicle is valid
  /** Carrier phase range solution status */
  enum CarrierPhaseRangeSolution : uint8_t
  {
    NONE = 0,      ///< No carrier phase range solution
    FLOATING = 1,  ///< Carrier phase range solution with floating ambiguities
    FIXED = 2,     ///< Carrier phase range solution with fixed ambiguities
  } carrSoln;

  // flags2: Additional flags
  bool confirmedAvai;  ///< Information about UTC Date and Time of Day validity confirmation is available
  bool confirmedDate;  ///< UTC Date validity could be confirmed
  bool confirmedTime;  ///< UTC Time of Day could be confirmed

  uint8_t numSV;  ///< Number of satellites used in Nav Solution

  double lon;      ///< Longitude [deg]
  double lat;      ///< Latitude [deg]
  int32_t height;  ///< Height above ellipsoid [mm]
  int32_t hMSL;    ///< Height above mean sea level [mm]
  uint32_t hAcc;   ///< Horizontal accuracy estimate [mm]
  uint32_t vAcc;   ///< Vertical accuracy estimate [mm]
  int32_t velN;    ///< NED north veloity [mm/s]
  int32_t velE;    ///< NED east veloity [mm/s]
  int32_t velD;    ///< NED down veloity [mm/s]
  int32_t gSpeed;  ///< Ground Speed (2-D) [mm/s]
  double headMot;  ///< Heading of motion (2-D) [deg]
  uint32_t sAcc;   ///< Speed accuracy estimate [mm/s]
  double headAcc;  ///< Heading accuracy estimate [deg]
  double pDOP;     ///< Position DOP

  // flags3: Additional flags
  bool invalidLlh;  ///< Invalid lon, lat, height and hMSL
  enum CorrectionAge : uint8_t  // Age of the most recently received differential correction
  {
    AGE_NOT_AVAILABLE = 0,
    AGE_0_TO_1_SEC = 1,
    AGE_1_TO_2_SEC = 2,
    AGE_2_TO_5_SEC = 3,
    AGE_5_TO_10_SEC = 4,
    AGE_10_TO_15_SEC = 5,
    AGE_15_TO_20_SEC = 6,
    AGE_20_TO_30_SEC = 7,
    AGE_30_TO_45_SEC = 8,
    AGE_45_TO_60_SEC = 9,
    AGE_60_TO_90_SEC = 10,
    AGE_90_TO_120_SEC = 11,
    AGE_120_SEC_OR_MORE = 12,
  } lastCorrectionAge;

  double headVeh;  ///< Heading of vehicle (2-D) [deg]
  double magDec;   ///< Magnetic declination [deg]
  double magAcc;   ///< Magnetic declination accuracy [deg]

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
  struct Satellite
  {
    uint8_t gnssId;  ///< GNSS identifier for assignment
    uint8_t svId;    ///< Satellite identifier for assignment
    uint8_t cno;     ///< Carrier to noise ratio [dB-Hz]
    int8_t elev;     ///< Elevation [deg]
    int16_t azim;    ///< Azimuth [deg]
    double prRes;    ///< Pseudorange residual [m]
    enum QualityIndicator : uint8_t
    {
      NO_SIGNAL = 0,
      SEARCHING_SIGNAL = 1,
      SIGNAL_ACQUIRED = 2,
      SIGNAL_DETECTED_BUT_UNUSABLE = 3,
      CODE_LOCKED_AND_TIME_SYNCHRONIZED = 4,
      CODE_AND_CARRIER_LOCKED_AND_TIME_SYNCHRONIZED = 5,
    } qualityInd;  ///< Signal quality indicator
    bool svUsed;   ///< Signal in the subset specified in Signal Identifiers is currently being used for navigation
    enum Health : uint8_t
    {
      UNKNOWN = 0,
      HEALTHY = 1,
      UNHEALTHY = 2,
    } health;       ///< Signal health flag
    bool diffCorr;  ///< Differential correction data is available for this SV
    bool smoothed;  ///< Carrier smoothed pseudorange used
    enum OrbitSource : uint8_t
    {
      NO_ORBIT_INFORMATION_AVAILABLE = 0,
      EPHEMERIS_USED = 1,
      ALMANAC_USED = 2,
      ASSIST_NOW_OFFLINE_ORBIT_USED = 3,
      ASSIST_NOW_AUTONOMOUS_ORBIT_USED = 4,
      OTHER_ORBIT_INFORMATION_USED = 5,
    } orbitSource;        ///< Orbit source
    bool ephAvail;        ///< Ephemeris is available for this SV
    bool almAvail;        ///< almanac is available for this SV
    bool anoAvail;        ///< AssistNow Offline data is available for this SV
    bool aopAvail;        ///< AssistNow Autonomous data is available for this SV
    bool sbasCorrUsed;    ///< SBAS corrections have been used
    bool rtcmCorrUsed;    ///< RTCM corrections have been used
    bool slasCorrUsed;    ///< QZSS SLAS corrections have been used
    bool spartnCorrUsed;  ///< SPARTN corrections have been used
    bool prCorrUsed;      ///< Pseudorange corrections have been used
    bool crCorrUsed;      ///< Carrier range corrections have been used
    bool doCorrUsed;      ///< Range rate (Doppler) corrections have been used
    bool clasCorrUsed;    ///< CLAS corrections have been used
  };

  uint32_t iTOW;    ///< GPS time of week [ms]
  uint8_t version;  ///< Message version
  uint8_t numSvs;   ///< Number of satellites

  std::vector<Satellite> satellites;

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
  struct Signal
  {
    uint8_t gnssId;  // GNSS identifier
    uint8_t svId;    // Satellite identifier
    uint8_t sigId;   // Signal identifier (meaning depends on `gnssId`)
    uint8_t freqId;  // GLONASS frequency slot + 7 (range from 0 to 13)
    double prRes;    // Pseudorange residual [m]
    uint8_t cno;     // Carrier-to-noise density ratio [dB-Hz]
    enum QualityIndicator : uint8_t
    {
      NO_SIGNAL = 0,
      SEARCHING_SIGNAL = 1,
      SIGNAL_ACQUIRED = 2,
      SIGNAL_DETECTED_BUT_UNUSABLE = 3,
      CODE_LOCKED_AND_TIME_SYNCHRONIZED = 4,
      CODE_AND_CARRIER_LOCKED_AND_TIME_SYNCHRONIZED_1 = 5,
      CODE_AND_CARRIER_LOCKED_AND_TIME_SYNCHRONIZED_2 = 6,
      CODE_AND_CARRIER_LOCKED_AND_TIME_SYNCHRONIZED_3 = 7,
    } qualityInd;  // Signal quality indicator
    enum CorrectionSource : uint8_t
    {
      NO_CORRECTIONS = 0,
      SBAS_CORRECTIONS = 1,
      BEIDOU_CORRECTIONS = 2,
      RTCM2_CORRECTIONS = 3,
      RTCM3_OSR_CORRECTIONS = 4,
      RTCM3_SSR_CORRECTIONS = 5,
      QZSS_SLAS_CORRECTIONS = 6,
      SPARTN_CORRECTIONS = 7,
      CLAS_CORRECTIONS = 8,
    } corrSource;  // Correction source
    uint8_t
      ionoModel;  // Ionospheric model used (0 = no model, 1 = Klobuchar GPS, 2 = SBAS, 3 = Klobuchar BeiDou, 8 = dual frequency)
    enum Health : uint8_t
    {
      UNKNOWN = 0,
      HEALTHY = 1,
      UNHEALTHY = 2,
    } health;         // Signal health flag
    bool prSmoothed;  // Pseudorange has been smoothed
    bool prUsed;      // Pseudorange has been used for this signal
    bool crUsed;      // Carrier range has been used for this signal
    bool doUsed;      // Range rate (Doppler) has been used for this signal
    bool prCorrUsed;  // Pseudorange corrections have been used for this signal
    bool crCorrUsed;  // Carrier range corrections have been used for this signal
    bool doCorrUsed;  // Range rate (Doppler) corrections have been used for this signal

    // Return true if both the code and the carrier phase are locked.
    inline bool isCarrierLocked() const;
  };

  uint32_t iTOW;    // GPS time of week [ms]
  uint8_t version;  // Message version
  uint8_t numSigs;  // Number of signals

  std::vector<Signal> signals;

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
  uint32_t iTOW;   ///< GPS time of week [ms]
  FixType gpsFix;  ///< GPSfix Type, this value does not qualify a fix as valid and within the limits

  // flags: Navigation Status Flags
  bool gpsFixOk;  ///< Position and velocity valid and within DOP and ACC Masks
  bool diffSoln;  ///< Differential corrections were applied
  bool wknSet;    ///< Week Number valid
  bool towSet;    ///< Time of Week valid

  // fixStat: Fix Status Information
  bool diffCorr;  ///< Differential corrections available
  /** Map matching status */
  enum MapMatchingStatus : uint8_t
  {
    NONE = 0b00,
    VALID_BUT_NOT_USED = 0b01,  ///< Valid but not used, i.e. map matching data was received, but was too old
    VALID_AND_USED_1 = 0b10,    ///< Valid and used, map matching data was applied
    /**
     * Valid and used, map matching data was applied.
     * In case of sensor unavailability map matching data enables dead reckoning.
     * This requires map matched latitude/longitude or heading data.
     */
    VALID_AND_USED_2 = 0b11,
  } mapMatching;

  // flags2: Further information about navigation output
  /** Power save mode state */
  enum PowerSaveModeState : uint8_t
  {
    ACQUISITION = 0,
    TRACKING = 1,
    POWER_OPTIMIXED_TRACKING = 2,
    INACTIVE = 3,
  } psmState;
  /** Spoofing detection state */
  enum SpoofDetectionState : uint8_t
  {
    UNKNOWN_OR_DEACTIVATED = 0,
    NO_SPOOFING_INDICATED = 1,
    SPOOFING_INDICATED = 2,
    MULTIPLE_SPOOFING_INDICATIONS = 3,
  } spoofDetState;

  enum CarrierPhaseRangeSolutionStatus : uint8_t
  {
    NO_SOLUTION = 0,
    FLOATING_AMBIGUITY = 1,
    FIXED_AMBIGUITY = 2,
  } carrSoln;

  uint32_t ttff;  ///< Time to first fix [ms]
  uint32_t msss;  ///< Milliseconds since Startup / Reset [ms]

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
  uint32_t iTOW;  ///< GPS time of week of the navigation epoch [ms]
  int32_t fTOW;   ///< Fractional part of iTOW [ns]
  int16_t week;   ///< GPS week number of the navigation epoch
  int8_t leapS;   ///< GPS leap seconds (GPS-UTC) [s]

  // valid: Validity Flags
  bool towValid;    ///< Valid GPS time of week
  bool weekValid;   ///< Valid GPS week number
  bool leapSValid;  ///< Valid GPS leap seconds

  uint32_t tAcc;  ///< Time Accuracy Estimate [ns]

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
  uint32_t iTOW;  ///< GPS time of week [ms]

  int32_t velN;  ///< North velocity component [cm/s]
  int32_t velE;  ///< East velocity component [cm/s]
  int32_t velD;  ///< Down velocity component [cm/s]

  uint32_t speed;   ///< Speed (3-D) [cm/s]
  uint32_t gSpeed;  ///< Ground Speed (2-D) [cm/s]
  double heading;   ///< Heading of motion 2-D [deg]

  uint32_t sAcc;  ///< Speed accuracy estimate [cm/s]
  double cAcc;    ///< Course / Heading accuracy estimate [deg]

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

struct RXM_RTCM : public Payload
{
  uint8_t version;      // Message version (0x02 for this version)
  uint8_t flags;        // RTCM input status flags (bit 0: crcFailed, bit 1-2: msgUsed)
  uint16_t subType;     // Message subtype (only applies for RTCM 4072 message)
  uint16_t refStation;  // Reference station ID
  uint16_t msgType;     // RTCM message type (e.g. 1005, 1075, 1115)

  bool crcFailed() const
  { return (flags & 0x01) != 0; }

  uint8_t msgUsed() const
  { return (flags >> 1) & 0x03; }

  void decode(const uint8_t* p) override;
  void print(std::ostream& os) const override;
};

inline bool NAV_SIG::Signal::isCarrierLocked() const
{ return qualityInd >= CODE_AND_CARRIER_LOCKED_AND_TIME_SYNCHRONIZED_1; }
}  // namespace payload
}  // namespace ublox
}  // namespace tobas