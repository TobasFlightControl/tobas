#pragma once

#include <string>
#include <tobas_std_tools/rate.hpp>
#include <tobas_std_tools/stopwatch.hpp>

#include "./spi_dev.hpp"
#include "./ubx_payload.hpp"

#define PACKED __attribute__((__packed__))  // 構造体のメンバ変数がメモリ上で連続する

namespace navio
{
static constexpr uint32_t kUbxBufferLength = 1024;
static constexpr uint32_t kPreambleOffset = 2;
static constexpr uint32_t kSpiSpeedHz = 5500000;  // Maximum frequency is 5.5MHz
static constexpr uint32_t kConfigureMessageSize = 11;
static constexpr uint32_t kMinMaxTrkChForMajorGnss = 4;
static constexpr double kWaitForGnssAck = 3.;  // [s]

// SPIで1バイト受け取る間隔 [us]
// 小さいほど通信遅延を小さくできるが，小さすぎるとレシーバへのリクエスト過多で精度が落ちる
// 50usだと明らかに位置精度が悪くなり，100usだと大きく改善 (2024/5/16)
// 100usの場合はPVT (92バイト) を受け取るのに 0.1 x 92 = 9.2ms かかる．
// PX4はデフォルトで 38400bit/s のUARTだから，PVTの受信遅延は 92 / (38400 / 8) * 1000 ~ 19.2ms．
// どうせ衛生からの通信遅延が70msで固定で，レシーバとFCの通信遅延はボトルネックではないから，速度より精度を重視すべき．
static constexpr size_t kSpiInterval = 100;

class UBXScanner
{
public:
  enum State
  {
    Sync1,
    Sync2,
    Class,
    ID,
    Length1,
    Length2,
    Payload,
    CK_A,
    CK_B,
    Done,
  };

  explicit UBXScanner();

  inline uint8_t* getMessage();
  inline const uint32_t& getMessageLength() const;
  inline const uint32_t& getPosition() const;

  void reset();
  int update(const uint8_t& data);

private:
  uint8_t message_[kUbxBufferLength];  // Buffer for UBX message
  uint32_t message_length_;            // Length of the received message
  uint32_t position_;                  // Indicates current buffer offset
  uint32_t payload_length_;            // Length of current message payload
  State state_;                        // Current scanner state
};

class UBXParser
{
public:
  explicit UBXParser(UBXScanner& ubxsc);

  uint16_t calcId();

  inline uint8_t* getMessage() const;
  inline const uint32_t& getLength() const;
  inline const uint32_t& getPosition() const;
  inline const uint16_t& getLatestMsg() const;

private:
  UBXScanner scanner_;  // The scanner, which finds the messages in the data stream

  uint8_t* message_;  // Pointer to the scanner's message buffer
  uint16_t latest_id_;
};

/**
 * @brief Ublox handler.
 * datasheet:
 * https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
 */
class Ublox
{
private:
  enum ubx_protocol_bytes : uint8_t
  {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,

    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    CLASS_MON = 0x0A,

    ID_NAV_POSLLH = 0x02,
    ID_NAV_STATUS = 0x03,
    ID_NAV_DOP = 0x04,
    ID_NAV_PVT = 0x07,
    ID_NAV_VELNED = 0x12,
    ID_NAV_TIMEGPS = 0x20,
    ID_NAV_TIMEUTC = 0x21,
    ID_NAV_COV = 0x36,

    ID_ACK_NAK = 0x00,
    ID_ACK_ACK = 0x01,

    ID_CFG_MSG = 0x01,
    ID_CFG_CFG = 0x09,
    ID_CFG_RATE = 0x08,
    ID_CFG_NAV5 = 0x24,
    ID_CFG_GNSS = 0x3E,
    ID_CFG_PMS = 0x86,

    ID_MON_HW = 0x09,
    ID_MON_HW2 = 0x0B,
  };

public:
  // Class + ID
  enum message_t : uint16_t
  {
    NAV_POSLLH = (CLASS_NAV << 8) + ID_NAV_POSLLH,
    NAV_STATUS = (CLASS_NAV << 8) + ID_NAV_STATUS,
    NAV_DOP = (CLASS_NAV << 8) + ID_NAV_DOP,
    NAV_PVT = (CLASS_NAV << 8) + ID_NAV_PVT,
    NAV_VELNED = (CLASS_NAV << 8) + ID_NAV_VELNED,
    NAV_TIMEGPS = (CLASS_NAV << 8) + ID_NAV_TIMEGPS,
    NAV_TIMEUTC = (CLASS_NAV << 8) + ID_NAV_TIMEUTC,
    NAV_COV = (CLASS_NAV << 8) + ID_NAV_COV,

    ACK_NAK = (CLASS_ACK << 8) + ID_ACK_NAK,
    ACK_ACK = (CLASS_ACK << 8) + ID_ACK_ACK,

    MON_HW = (CLASS_MON << 8) + ID_MON_HW,
    MON_HW2 = (CLASS_MON << 8) + ID_MON_HW2,
  };

  // gpsFix (UBX-STATUS), fixType (UBX-PVT)
  enum gps_fix : uint8_t
  {
    NO_FIX = 0,
    DEAD_RECHONING_ONLY = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    GPS_DEAD_RECHONING_COMBINED = 4,
    TIME_ONLY_FIX = 5,
  };

  // dynModel (UBX-CFG-NAV5)
  enum dynamics_model : uint8_t
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
    ELECTRIC_KICK_SCOOTER = 12,
  };

  // powerSetupValue (UBX-CFG-PMS)
  enum power_setup_value : uint8_t
  {
    FULL_POWER = 0,
    BALANCED = 1,
    INTERVAL = 2,
    AGGRESSIVE_WITH_1HZ = 3,
    AGGRESSIVE_WITH_2HZ = 4,
    AGGRESSIVE_WITH_4HZ = 5,
  };

  // gnssId (UBX-CFG-GNSS)
  enum gnss_id : uint8_t
  {
    GPS = 0,
    SBAS = 1,
    GALILEO = 2,
    BEIDOU = 3,
    QZSS = 5,
    GLONASS = 6,
  };

  explicit Ublox();

  /* 32.10.3.1 Clear, save and load configurations */
  void clearConfigurations();
  void saveConfigurations();
  void loadConfigurations();

  /* 32.10.18.3 Set message rate */
  void enableMsg(message_t msg, bool enable);
  void enableAllMsgs(bool enable);

  /* 32.10.27.1 Navigation/measurement rate settings */
  void configureSolutionRate(uint16_t meas_rate, uint16_t nav_rate = 1, uint16_t time_ref = 1);

  /* 32.10.19.1 Navigation engine settings */
  void configureDynamicsModel(dynamics_model dyn_model);

  /* 32.10.24.1 Power mode setup */
  void configurePowerMode(power_setup_value mode, uint16_t period = 0, uint16_t on_time = 0);

  /* 32.10.13.1 GNSS system configuration */
  void configureGnss_GPS(bool enable, uint8_t res_track_ch = 8, uint8_t max_track_ch = 16);
  void configureGnss_SBAS(bool enable, uint8_t res_track_ch = 1, uint8_t max_track_ch = 3);
  void configureGnss_Galileo(bool enable, uint8_t res_track_ch = 4, uint8_t max_track_ch = 8);
  void configureGnss_BeiDou(bool enable, uint8_t res_track_ch = 8, uint8_t max_track_ch = 16);
  void configureGnss_QZSS(bool enable, uint8_t res_track_ch = 0, uint8_t max_track_ch = 3);
  void configureGnss_GLONASS(bool enable, uint8_t res_track_ch = 8, uint8_t max_track_ch = 14);

  uint16_t update();

  void decode(NavPosllhPayload& data) const;
  void decode(NavStatusPayload& data) const;
  void decode(NavDopPayload& data) const;
  void decode(NavPvtPayload& data) const;
  void decode(NavVelnedPayload& data) const;
  void decode(NavTimegpsPayload& data) const;
  void decode(NavTimeutcPayload& data) const;
  void decode(NavCovPayload& data) const;

  void decode(AckNakPayload& data) const;
  void decode(AckAckPayload& data) const;

  void decode(MonHwPayload& data) const;
  void decode(MonHw2Payload& data) const;

private:
  struct PACKED UbxHeader
  {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
  };

  struct PACKED CheckSum
  {
    uint8_t CK_A;
    uint8_t CK_B;
  };

  /* ===== Payload structures ===== */
  struct PACKED CfgCfg
  {
    uint32_t clearMask;
    uint32_t saveMask;
    uint32_t loadMask;
  };

  struct PACKED CfgMsg
  {
    uint8_t msgClass;
    uint8_t msgID;
    uint8_t rate;
  };

  struct PACKED CfgRate
  {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
  };

  struct PACKED CfgNav5
  {
    uint16_t mask;
    uint8_t dynModel;
    uint8_t fixMode;
    int32_t fixedAlt;
    uint32_t fixedAltVar;
    int8_t minElev;
    uint8_t drLimit;
    uint16_t pDop;
    uint16_t tDop;
    uint16_t pAcc;
    uint16_t tAcc;
    uint8_t staticHoldThresh;
    uint8_t dgnssTimeout;
    uint8_t cnoThreshNumSVs;
    uint8_t cnoThresh;
    const uint8_t reserved1[2] = { 0, 0 };
    uint16_t staticHoldMaxDist;
    uint8_t utcStandard;
    const uint8_t reserved2[5] = { 0, 0, 0, 0, 0 };
  };

  struct PACKED CfgPms
  {
    const uint8_t version = 0x00;
    uint8_t powerSetupValue;
    uint16_t period;
    uint16_t onTime;
    const uint8_t reserved1[2] = { 0, 0 };
  };

  struct PACKED CfgGnssBlock
  {
    uint8_t gnssId;
    uint8_t resTrkCh;
    uint8_t maxTrkCh;
    const uint8_t reserved1 = 0;
    uint32_t flags;
  };

  struct PACKED CfgGnss
  {
    const uint8_t msgVer = 0x00;
    const uint8_t numTrkChHw = 0;  // Read only
    uint8_t numTrkChUse;
    const uint8_t numConfigBlocks = 1;
    CfgGnssBlock block;
  };
  /* ==============================*/

  SPIdev spi_dev_;
  UBXScanner scanner_;
  UBXParser parser_;

  tobas_std::Rate rate_;
  tobas_std::Stopwatch stopwatch_;

  void sendMessage(uint8_t msg_class, uint8_t msg_id, void* msg, uint16_t size);
  void waitForAcknowledge(uint8_t cls, uint8_t id);
  void configure(uint8_t cfg_id, void* msg, uint16_t size);

  void configureGnss(uint8_t gnss_id, uint8_t res_track_ch, uint8_t max_track_ch, bool enable);

  /* p.171, 32.4 UBX Checksum. */
  static CheckSum calculateCheckSum(uint8_t* message, size_t size);
  static int spliceMemory(uint8_t* dest, const void* const src, size_t size, int dest_offset = 0);
};

inline uint8_t* UBXScanner::getMessage()
{
  return message_;
}

inline const uint32_t& UBXScanner::getMessageLength() const
{
  return message_length_;
}

inline const uint32_t& UBXScanner::getPosition() const
{
  return position_;
}

inline uint8_t* UBXParser::getMessage() const
{
  return message_;
}

inline const uint32_t& UBXParser::getLength() const
{
  return scanner_.getMessageLength();
}

inline const uint32_t& UBXParser::getPosition() const
{
  return scanner_.getPosition();
}

inline const uint16_t& UBXParser::getLatestMsg() const
{
  return latest_id_;
}
}  // namespace navio
