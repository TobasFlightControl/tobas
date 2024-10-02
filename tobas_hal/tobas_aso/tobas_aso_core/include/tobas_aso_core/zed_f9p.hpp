#pragma once

#include <tobas_std_tools/rate.hpp>
#include <tobas_std_tools/stopwatch.hpp>
#include <tobas_linux/spi_dev.hpp>

#include "./ubx_scanner.hpp"
#include "./ubx_payload.hpp"

#define PACKED __attribute__((__packed__))  // 構造体のメンバ変数がメモリ上で連続する

namespace aso
{
/**
 * @brief A Linux driver of u-blox ZED-F9P-04B using SPI interface and UBX protocol.
 *
 * Product Page: https://www.u-blox.com/en/product/zed-f9p-module
 * Datasheet: https://content.u-blox.com/sites/default/files/ZED-F9P-04B_DataSheet_UBX-21044850.pdf
 * Interface Description:
 * https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf
 * ANN-MB series antenna: https://www.u-blox.com/en/product/ann-mb-series
 */
class ZEDF9P
{
private:
  static constexpr uint32_t kSpiClockFreq = 5'500'000;  // Maximum frequency is 5.5MHz
  static constexpr uint8_t kRG174CableDelay = 5;        // [ns/m] 同軸ケーブルの遅延
  static constexpr double kWaitForGnssAck = 1.;         // [s]

  // SPIで1バイト受け取る間隔 [us]
  // 小さいほど通信遅延を小さくできるが，小さすぎるとレシーバへのリクエスト過多で精度が落ちる．
  // 少なくとも50usまでは下げられるが，CPU負荷を抑えるため大きめにしておく．
  static constexpr auto kReqInterval = std::chrono::microseconds(100);

public:
  enum ubx_class_t : uint8_t
  {
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    CLASS_INF = 0x04,
    CLASS_LOG = 0x21,
    CLASS_MGA = 0x13,
    CLASS_MON = 0x0A,
    CLASS_NAV = 0x01,
    CLASS_SEC = 0x27,
    CLASS_TIM = 0x0D,
    CLASS_UPD = 0x09,
  };

  enum ubx_ack_id_t : uint8_t
  {
    ACK_ACK = 0x01,
    ACK_NAK = 0x00,
  };

  enum ubx_cfg_id_t : uint8_t
  {
    CFG_VALDEL = 0x8C,
    CFG_VALGET = 0x8B,
    CFG_VALSET = 0x8A,
  };

  enum ubx_nav_id_t : uint8_t
  {
    NAV_CLOCK = 0x22,      // Clock solution
    NAV_COV = 0x36,        // Covariance matrices
    NAV_DOP = 0x04,        // Dilution of precision
    NAV_EOE = 0x61,        // End of epoch
    NAV_GEOFENCE = 0x39,   // Geofencing status
    NAV_HPPOSECEF = 0x13,  // High precision position solution in ECEF
    NAV_HPPOSLLH = 0x14,   // High precision geodetic position solution
    NAV_ODO = 0x09,        // Odometer solution
    NAV_ORB = 0x34,        // GNSS orbit database info
    NAV_PL = 0x62,         // Protection level information
    NAV_POSECEF = 0x01,    // Position solution in ECEF
    NAV_POSLLH = 0x02,     // Geodetic position solution
    NAV_PVT = 0x07,        // Navigation position velocity time solution
    NAV_RELPOSNED = 0x3C,  // Relative positioning information in NED frame
    NAV_RESETODO = 0x10,   // Reset odometer
    NAV_SAT = 0x35,        // Satellite information
    NAV_SBAS = 0x32,       // SBAS status data
    NAV_SIG = 0x43,        // Signal information
    NAV_SLAS = 0x42,       // QZSS L1S SLAS status data
    NAV_STATUS = 0x03,     // Receiver navigation status
    NAV_SVIN = 0x3B,       // Survey-in data
    NAV_TIMEBDS = 0x24,    // BeiDou time solution
    NAV_TIMEGAL = 0x25,    // Galileo time solution
    NAV_TIMEGLO = 0x23,    // GLONASS time solution
    NAV_TIMEGPS = 0x20,    // GPS time solution
    NAV_TIMELS = 0x26,     // Leap second event information
    NAV_TIMEQZSS = 0x27,   // QZSS time solution
    NAV_TIMEUTC = 0x21,    // UTC time solution
    NAV_VELECEF = 0x11,    // Velocity solution in ECEF
    NAV_VELNED = 0x12,     // Velocity solution in NED frame
  };

  /* Constants for CFG-NAVSPG-DYNMODEL */
  enum dynamics_model_t : uint8_t
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

  /* SPI Protocol Key ID */
  enum cfg_protocol_t : uint8_t
  {
    UBX = 0x01,
    NMEA = 0x02,
    RTCM3X = 0x04,
    SPARTN = 0x05,
  };

  explicit ZEDF9P();

  bool initialize();
  bool update();

  /* ===== Configurations =====*/

  bool enableMsg(ubx_class_t cls, uint8_t id, bool enable);
  bool configureDynamicsModel(dynamics_model_t model);
  bool configureMeasurementRate(uint16_t period_ms);

  bool enableGPS(bool enable);
  bool enableSBAS(bool enable);
  bool enableGalileo(bool enable);
  bool enableBeiDou(bool enable);
  bool enableQZSS(bool enable);
  bool enableGLONASS(bool enable);

  bool enableProtocol(cfg_protocol_t prot, bool enable);

  /* RF174ケーブルの長さからアナログ伝達の遅延を設定する． */
  bool setAntennaLength(uint8_t length_m);

  bool enableUSB(bool enable);

  /* ===== Getters ===== */

  inline ubx_class_t latestClass() const;
  inline uint8_t latestId() const;

  inline const uint8_t* payload() const;

private:
  /* Supported storage size identifiers */
  enum cfg_size_t : uint8_t
  {
    ONE_BIT = 0x01,  // Only the LSB is used
    ONE_BYTE = 0x02,
    TWO_BYTES = 0x03,
    FOUR_BYTES = 0x04,
    EIGHT_BYTES = 0x05,
  };

  enum cfg_group_t : uint8_t
  {
    CFG_BDS = 0x34,           // BeiDou System Configuration
    CFG_GEOFENCE = 0x24,      // Geofencing Configuration
    CFG_HW = 0xA3,            // Hardware Configuration
    CFG_I2C = 0x51,           // Configuration of the I2C Interface
    CFG_I2CINPROT = 0x71,     // Input Protocol Configuration of the I2C Interface
    CFG_I2COUTPROT = 0x72,    // Output Protocol Configuration of the I2C Interface
    CFG_INFMSG = 0x92,        // Inf Message Configuration
    CFG_ITFM = 0x41,          // Jamming/Interference Monitor configuration
    CFG_LOGFILTER = 0xDE,     // Data Logger Configuration
    CFG_MOT = 0x25,           // Motion Detector Configuration
    CFG_MSGOUT = 0x91,        // Message Output Configuration
    CFG_NAV2 = 0x17,          // Secondary output configuration
    CFG_NAVHPG = 0x14,        // High Precision Navigation Configuration
    CFG_NAVSPG = 0x11,        // Standard Precision Navigation Configuration
    CFG_NMEA = 0x93,          // NMEA Protocol Configuration
    CFG_ODO = 0x22,           // Odometer and Low-Speed Course Over Ground Filter Configuration
    CFG_QZSS = 0x37,          // QZSS System Configuration
    CFG_RATE = 0x21,          // Navigation and Measurement Rate Configuration
    CFG_RINV = 0xC7,          // Remote Inventory
    CFG_RTCM = 0x09,          // RTCM Protocol Configuration
    CFG_SBAS = 0x36,          // SBAS Configuration
    CFG_SEC = 0xF6,           // Security Configuration
    CFG_SIGNAL = 0x31,        // Satellite Systems (GNSS) Signal Configuration
    CFG_SPARTN = 0xA7,        // SPARTN Configuration
    CFG_SPI = 0x64,           // Configuration of the SPI Interface
    CFG_SPIINPROT = 0x79,     // Input Protocol Configuration of the SPI Interface
    CFG_SPIOUTPROT = 0x7A,    // Output Protocol Configuration of the SPI Interface
    CFG_TMODE = 0x03,         // Time Mode Configuration
    CFG_TP = 0x05,            // Timepulse Configuration
    CFG_TXREADY = 0xA2,       // Tx-Ready Configuration
    CFG_URART1 = 0x52,        // Configuration of the UART1 Interface
    CFG_UART1INPROT = 0x73,   // Input Protocol Configuration of the UART1 Interface
    CFG_UART1OUTPROT = 0x74,  // Output Protocol Configuration of the UART1 Interface
    CFG_URART2 = 0x53,        // Configuration of the UART2 Interface
    CFG_UART2INPROT = 0x75,   // Input Protocol Configuration of the UART2 Interface
    CFG_UART2OUTPROT = 0x76,  // Output Protocol Configuration of the UART2 Interface
    CFG_USB = 0x65,           // Configuration of the USB Interface
    CFG_USBINPROT = 0x77,     // Input Protocol Configuration of the USB Interface
    CFG_USBOUTPROT = 0x78,    // Output Protocol Configuration of the USB Interface
  };

  struct PACKED UbxHeader
  {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t cls;
    uint8_t id;
    uint16_t length;
  };

  struct PACKED CheckSum
  {
    uint8_t CK_A;
    uint8_t CK_B;
  };

  /* ===== Payload structures ===== */
  template <typename ValueType>
  struct PACKED CfgData
  {
    uint32_t key;     // Configuration Key ID
    ValueType value;  // Configuration Value
  };

  template <typename ValueType, size_t N>
  struct PACKED CfgValSet
  {
    const uint8_t version = 0x00;  // Message version, set to 0

    // The layers where the configuration should be applied
    const enum cfg_layer_t : uint8_t {
      RAM = 0b001,
      BBR = 0b010,
      FLASH = 0b100,
    } layers = RAM;

    const uint8_t reserved1[2] = { 0 };  // Reserved

    CfgData<ValueType> data[N];  // Configuration data
  };
  /* ==============================*/

  linux::SPIdev spi_dev_;
  UBXScanner scanner_;

  tobas_std::Rate rate_;
  tobas_std::Stopwatch stopwatch_;

  bool sendMessage(ubx_class_t cls, uint8_t id, const void* msg, uint16_t size);
  bool waitForAcknowledge(ubx_class_t cls, uint8_t id);
  bool configure(ubx_cfg_id_t cfg_id, const void* msg, uint16_t size);
  bool verifyMessage() const;

  /* 5.4 UBX Checksum */
  static CheckSum computeChecksum(const uint8_t* message, size_t checksum_pos);

  static size_t spliceMemory(uint8_t* dest, const void* src, size_t size, size_t dest_offset = 0);

  /* Configuration Key ID | 6.2 Configuration Items */
  static uint32_t configKeyID(cfg_size_t size, cfg_group_t group, uint8_t id);
};

inline ZEDF9P::ubx_class_t ZEDF9P::latestClass() const
{
  return static_cast<ubx_class_t>(*scanner_.getClass());
}

inline uint8_t ZEDF9P::latestId() const
{
  return *scanner_.getId();
}

inline const uint8_t* ZEDF9P::payload() const
{
  return scanner_.getPayload();
}
}  // namespace aso
