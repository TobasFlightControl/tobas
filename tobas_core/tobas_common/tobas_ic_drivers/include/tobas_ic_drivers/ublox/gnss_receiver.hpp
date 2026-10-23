// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "./ubx_payload.hpp"
#include "./ubx_scanner.hpp"
#include "./ubx_transport.hpp"

#define PACKED __attribute__((__packed__))  // Struct member variables are contiguous in memory.

namespace tobas::ublox
{
/** A Linux driver of u-blox GNSS receivers using UBX protocol. */
class GnssReceiver
{
public:
  enum UbxClass : uint8_t
  {
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    CLASS_INF = 0x04,
    CLASS_LOG = 0x21,
    CLASS_MGA = 0x13,
    CLASS_MON = 0x0A,
    CLASS_NAV = 0x01,
    CLASS_NAV2 = 0x29,
    CLASS_RXM = 0x02,
    CLASS_SEC = 0x27,
    CLASS_TIM = 0x0D,
    CLASS_UPD = 0x09,
  };

  enum UbxAckId : uint8_t
  {
    ACK_ACK = 0x01,
    ACK_NAK = 0x00,
  };

  enum UbxCfgId : uint8_t
  {
    CFG_VALDEL = 0x8C,
    CFG_VALGET = 0x8B,
    CFG_VALSET = 0x8A,
  };

  enum UbxMonId : uint8_t
  {
    MON_COMMS = 0x36,
    MON_GNSS = 0x28,
    MON_HW = 0x09,
    MON_HW2 = 0x0B,
    MON_HW3 = 0x37,
    MON_IO = 0x02,
    MON_MSGPP = 0x06,
    MON_PATCH = 0x27,
    MON_POST = 0x3B,
    MON_RF = 0x38,
    MON_RXBUF = 0x07,
    MON_RXR = 0x21,
    MON_SPAN = 0x31,
    MON_SYS = 0x39,
    MON_TXBUF = 0x08,
    MON_VER = 0x04,
  };

  enum UbxNavId : uint8_t
  {
    NAV_CLOCK = 0x22,      ///< Clock solution
    NAV_COV = 0x36,        ///< Covariance matrices
    NAV_DOP = 0x04,        ///< Dilution of precision
    NAV_EOE = 0x61,        ///< End of epoch
    NAV_GEOFENCE = 0x39,   ///< Geofencing status
    NAV_HPPOSECEF = 0x13,  ///< High precision position solution in ECEF
    NAV_HPPOSLLH = 0x14,   ///< High precision geodetic position solution
    NAV_ODO = 0x09,        ///< Odometer solution
    NAV_ORB = 0x34,        ///< GNSS orbit database info
    NAV_PL = 0x62,         ///< Protection level information
    NAV_POSECEF = 0x01,    ///< Position solution in ECEF
    NAV_POSLLH = 0x02,     ///< Geodetic position solution
    NAV_PVT = 0x07,        ///< Navigation position velocity time solution
    NAV_RELPOSNED = 0x3C,  ///< Relative positioning information in NED frame
    NAV_RESETODO = 0x10,   ///< Reset odometer
    NAV_SAT = 0x35,        ///< Satellite information
    NAV_SBAS = 0x32,       ///< SBAS status data
    NAV_SIG = 0x43,        ///< Signal information
    NAV_SLAS = 0x42,       ///< QZSS L1S SLAS status data
    NAV_STATUS = 0x03,     ///< Receiver navigation status
    NAV_SVIN = 0x3B,       ///< Survey-in data
    NAV_TIMEBDS = 0x24,    ///< BeiDou time solution
    NAV_TIMEGAL = 0x25,    ///< Galileo time solution
    NAV_TIMEGLO = 0x23,    ///< GLONASS time solution
    NAV_TIMEGPS = 0x20,    ///< GPS time solution
    NAV_TIMELS = 0x26,     ///< Leap second event information
    NAV_TIMEQZSS = 0x27,   ///< QZSS time solution
    NAV_TIMEUTC = 0x21,    ///< UTC time solution
    NAV_VELECEF = 0x11,    ///< Velocity solution in ECEF
    NAV_VELNED = 0x12,     ///< Velocity solution in NED frame
  };

  /** Constants for CFG-NAVSPG-DYNMODEL */
  enum DynamicsModel : uint8_t
  {
    PORTABLE = 0,
    STATIONARY = 2,
    PEDESTRIAN = 3,
    AUTOMOTIVE = 4,
    SEA = 5,
    AIRBORNE_1G = 6,
    AIRBORNE_2G = 7,
    AIRBORNE_4G = 8,
    WRIST_WORN_WATCH = 9,
    MOTORBIKE = 10,
    ROBOTIC_LAWN_MOWER = 11,
    E_SCOOTER = 12,
  };

  enum ReceiverProfile
  {
    F9P,
    X20P,
  };

  explicit GnssReceiver(std::unique_ptr<UbxTransport> _transport);

  GnssReceiver(GnssReceiver&& _other) = delete;
  GnssReceiver& operator=(GnssReceiver&& _other) = delete;
  GnssReceiver(const GnssReceiver& _other) = delete;
  GnssReceiver& operator=(const GnssReceiver& _other) = delete;

  bool initialize();
  bool update(bool _blocking = false);

  // ===== Configurations =====

  bool enableMessage(UbxClass _cls, uint8_t _id, bool _enable);
  bool configureDynamicsModel(DynamicsModel _model);
  bool configureMeasurementRate(uint16_t _period_ms);

  bool enableGps();
  bool disableGps();

  bool enableSbas();
  bool disableSbas();

  bool enableGalileo();
  bool disableGalileo();

  bool enableBeiDou();
  bool disableBeiDou();

  bool enableQzss();
  bool disableQzss();

  bool enableGlonass();
  bool disableGlonass();

  bool enableNavIc();
  bool disableNavIc();

  bool enableProtocol_UBX(bool _enable_input, bool _enable_output);
  bool enableProtocol_NMEA(bool _enable_input, bool _enable_output);
  bool enableProtocol_RTCM3X(bool _enable_input, bool _enable_output);
  bool enableProtocol_SPARTN(bool _enable_input);
  /** Set the analog transmission delay from the length of the `RF174` cable. */
  bool setAntennaLength(uint8_t _length_m);

  bool enableUsb(bool _enable);

  // ===== Getters =====

  inline UbxClass latestClass() const;
  inline uint8_t latestId() const;

  inline const uint8_t* payload() const;

private:
  /** Supported storage size identifiers */
  enum CfgSize : uint8_t
  {
    ONE_BIT = 0x01,  ///< Only the LSB is used.
    ONE_BYTE = 0x02,
    TWO_BYTES = 0x03,
    FOUR_BYTES = 0x04,
    EIGHT_BYTES = 0x05,
  };

  enum CfgGroup : uint8_t
  {
    CFG_BDS = 0x34,           ///< BeiDou System Configuration
    CFG_GEOFENCE = 0x24,      ///< Geofencing Configuration
    CFG_HW = 0xA3,            ///< Hardware Configuration
    CFG_I2C = 0x51,           ///< Configuration of the I2C Interface
    CFG_I2CINPROT = 0x71,     ///< Input Protocol Configuration of the I2C Interface
    CFG_I2COUTPROT = 0x72,    ///< Output Protocol Configuration of the I2C Interface
    CFG_INFMSG = 0x92,        ///< Inf Message Configuration
    CFG_LOGFILTER = 0xDE,     ///< Data Logger Configuration
    CFG_MOT = 0x25,           ///< Motion Detector Configuration
    CFG_MSGOUT = 0x91,        ///< Message Output Configuration
    CFG_NAV2 = 0x17,          ///< Secondary output configuration
    CFG_NAVHPG = 0x14,        ///< High Precision Navigation Configuration
    CFG_NAVSPG = 0x11,        ///< Standard Precision Navigation Configuration
    CFG_NMEA = 0x93,          ///< NMEA Protocol Configuration
    CFG_ODO = 0x22,           ///< Odometer and Low-Speed Course Over Ground Filter Configuration
    CFG_QZSS = 0x37,          ///< QZSS System Configuration
    CFG_RATE = 0x21,          ///< Navigation and Measurement Rate Configuration
    CFG_RINV = 0xC7,          ///< Remote Inventory
    CFG_RTCM = 0x09,          ///< RTCM Protocol Configuration
    CFG_SBAS = 0x36,          ///< SBAS Configuration
    CFG_SEC = 0xF6,           ///< Security Configuration
    CFG_SIGNAL = 0x31,        ///< Satellite Systems (GNSS) Signal Configuration
    CFG_SPARTN = 0xA7,        ///< SPARTN Configuration
    CFG_SPI = 0x64,           ///< Configuration of the SPI Interface
    CFG_SPIINPROT = 0x79,     ///< Input Protocol Configuration of the SPI Interface
    CFG_SPIOUTPROT = 0x7A,    ///< Output Protocol Configuration of the SPI Interface
    CFG_TMODE = 0x03,         ///< Time Mode Configuration
    CFG_TP = 0x05,            ///< Timepulse Configuration
    CFG_TXREADY = 0xA2,       ///< Tx-Ready Configuration
    CFG_URART1 = 0x52,        ///< Configuration of the UART1 Interface
    CFG_UART1INPROT = 0x73,   ///< Input Protocol Configuration of the UART1 Interface
    CFG_UART1OUTPROT = 0x74,  ///< Output Protocol Configuration of the UART1 Interface
    CFG_URART2 = 0x53,        ///< Configuration of the UART2 Interface
    CFG_UART2INPROT = 0x75,   ///< Input Protocol Configuration of the UART2 Interface
    CFG_UART2OUTPROT = 0x76,  ///< Output Protocol Configuration of the UART2 Interface
    CFG_USB = 0x65,           ///< Configuration of the USB Interface
    CFG_USBINPROT = 0x77,     ///< Input Protocol Configuration of the USB Interface
    CFG_USBOUTPROT = 0x78,    ///< Output Protocol Configuration of the USB Interface
  };

  /** Protocol Key ID */
  enum CfgProtocol : uint8_t
  {
    UBX = 0x01,
    NMEA = 0x02,
    RTCM3X = 0x04,
    SPARTN = 0x05,
  };

  // ===== Payload structures =====

  template <typename ValueType>
  struct PACKED CfgData
  {
    uint32_t key;     ///< Configuration Key ID
    ValueType value;  ///< Configuration Value
  };

  template <typename ValueType, size_t N>
  struct PACKED CfgValSet
  {
    const uint8_t version = 0x00;  ///< Message version, set to 0

    /** The layers where the configuration should be applied. */
    enum CfgLayer : uint8_t
    {
      RAM = 0b001,
      BBR = 0b010,
      FLASH = 0b100,
    } const layers = RAM;

    const uint8_t reserved1[2] = {};  ///< Reserved

    CfgData<ValueType> data[N];  ///< Configuration data
  };

  // ==============================

  const std::unique_ptr<UbxTransport> transport_;
  UbxScanner scanner_;
  std::optional<ReceiverProfile> receiver_profile_;

  template <typename T>
  bool cfgValSetSingle(CfgSize _size, CfgGroup _group, uint8_t _id, T _value);

  std::expected<ReceiverProfile, std::string> detectReceiverProfile();
  bool waitForAcknowledge(UbxClass _cls, uint8_t _id);
  bool configure(UbxCfgId _cfg_id, const void* _msg, uint16_t _size);

  bool enableGps(bool _enable);
  bool enableGpsL1();
  bool enableGpsL2();
  bool enableGpsL5();

  bool enableSbas(bool _enable);
  bool enableSbasL1();

  bool enableGalileo(bool _enable);
  bool enableGalileoE1();
  bool enableGalileoE5b();
  bool enableGalileoE5a();
  bool enableGalileoE6();

  bool enableBeiDou(bool _enable);
  bool enableBeiDouB1I();
  bool enableBeiDouB1C();
  bool enableBeiDouB2I();
  bool enableBeiDouB3I();
  bool enableBeiDouB2a();

  bool enableQzss(bool _enable);
  bool enableQzssL1();
  bool enableQzssL2();
  bool enableQzssL5();

  bool enableGlonass(bool _enable);
  bool enableGlonassL1();
  bool enableGlonassL2();

  bool enableNavIc(bool _enable);
  bool enableNavIcL5();

  bool enableInputProtocol(CfgProtocol _prot, bool _enable);
  bool enableOutputProtocol(CfgProtocol _prot, bool _enable);

  /** Configuration Key ID | 6.2 Configuration Items */
  static uint32_t configKeyID(CfgSize _size, CfgGroup _group, uint16_t _id);
};

inline GnssReceiver::UbxClass GnssReceiver::latestClass() const
{
  return static_cast<UbxClass>(*scanner_.getClass());
}

inline uint8_t GnssReceiver::latestId() const
{
  return *scanner_.getId();
}

inline const uint8_t* GnssReceiver::payload() const
{
  return scanner_.getPayload();
}

template <typename T>
bool GnssReceiver::cfgValSetSingle(CfgSize _size, CfgGroup _group, uint8_t _id, T _value)
{
  CfgValSet<T, 1> cfg;
  cfg.data[0].key = configKeyID(_size, _group, _id);
  cfg.data[0].value = _value;
  if (!configure(CFG_VALSET, &cfg, sizeof(cfg))) {
    return false;
  }
  // CFG-SIGNAL can reset the GNSS subsystem; wait after its ACK before another command.
  if (_group == CFG_SIGNAL) {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  return true;
}
}  // namespace tobas::ublox

#undef PACKED
