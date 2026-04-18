#pragma once

#include <cstdint>

#include <tobas_linux/i2c_dev.hpp>

namespace tobas
{
namespace driver
{
/**
 * @brief A linux driver of 3-axis magnetometer bmm350.
 *
 * Datasheet: https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm350/
 */
class BMM350
{
  static constexpr char kI2cDevice[] = "/dev/i2c-1";
  static constexpr uint8_t kI2cAddress = 0x14;
  // static constexpr double kHighFieldThresholdUt = 2000.0;

public:
  explicit BMM350();

  enum Odr : uint8_t
  {
    ODR_100Hz = 0x04,
    ODR_25Hz = 0x06,
  };

  enum Averaging : uint8_t
  {
    AVG_2 = 0x01,
    AVG_4 = 0x04,
  };

  /** @brief センサ初期化を実行する. */
  bool initialize();

  /**
   * @brief 初期化時に適用するODRとaveragingを設定する．
   *
   * @param _odr ODR設定値
   * @param _averaging averaging設定値
   * @note 実機反映はinitialize()内のapplyConfiguration()内で行います．
   */
  bool configure(uint8_t _odr = ODR_100Hz, uint8_t _averaging = AVG_4);
  uint8_t odr_;
  uint8_t averaging_;

  /** @brief 現在の磁気データ [uT] を読み出す. */
  bool readMag(double& _mx, double& _my, double& _mz);

  /** @brief 高磁界を判定し、必要なら磁気リセットを行う. */
  // bool checkForHighfieldAndReset(double _mx, double _my, double _mz);

private:
  enum Register : uint8_t
  {

    WHO_AM_I_REG = 0x00,
    PAD_CTRL_REG = 0x03,
    PMU_CMD_AGGR_SET_REG = 0x04,
    PMU_CMD_AXIS_EN_REG = 0x05,
    PMU_CMD_REG = 0x06,
    PMU_CMD_STATUS_0_REG = 0x07,
    OUT_X_XLSB_REG = 0x31,
    OTP_CMD_REG = 0x50,
    OTP_DATA_MSB_REG = 0x52,
    OTP_DATA_LSB_REG = 0x53,
    OTP_STATUS_REG = 0x55,
    TMR_SELFTEST_USER_REG = 0x60,
    CMD_REG = 0x7E,
  };

  enum ChipId : uint8_t
  {
    CHIP_ID = 0x33,
  };

  enum Config : uint8_t
  {
    SOFT_RESET_CMD = 0xB6,
    PMU_SUSPEND = 0x00,
    PMU_NORMAL = 0x01,
    PMU_UPD_OAE = 0x02,
    PMU_FM_FAST = 0x04,
    PMU_FGR = 0x05,
    PMU_BR = 0x07,
    PMU_BR_FAST = 0x08,
    PAD_DRIVE_STRONGEST = 0x07,
    ENABLE_XYZ = 0x07,
    OTP_DIR_READ = 0x20,
    OTP_PWR_OFF = 0x80,
  };

  enum PmuBits : uint8_t
  {
    AVG_MSK = 0x30,
    AVG_POS = 4,
    PMU_CMD_BUSY_MSK = 0x01,
    ODR_OVWR_MSK = 0x02,
    ODR_OVWR_POS = 1,
    AVG_OVWR_MSK = 0x04,
    AVG_OVWR_POS = 2,
    PWR_MODE_IS_NORMAL_MSK = 0x08,
    PWR_MODE_IS_NORMAL_POS = 3,
    CMD_IS_ILLEGAL_MSK = 0x10,
    CMD_IS_ILLEGAL_POS = 4,
    PMU_CMD_VALUE_MSK = 0xE0,
    PMU_CMD_VALUE_POS = 5,
  };

  enum OtpBits : uint8_t
  {
    OTP_WORD_ADDR_MSK = 0x1F,
    OTP_STATUS_CMD_DONE = 0x01,
    OTP_STATUS_ERROR_MSK = 0xE0,
  };
  enum BitMask : uint16_t
  {
    LSB_MASK = 0x00FF,
    MSB_MASK = 0xFF00,
  };

  enum SignBits : int
  {
    SIGNED_8_BIT = 8,
    SIGNED_12_BIT = 12,
    SIGNED_16_BIT = 16,
    SIGNED_24_BIT = 24,
  };

  // Bosch BMM350 SensorAPIのbmm350_defs.hの待ち時間に合わせています.
  enum TimingUs : uint32_t
  {
    STARTUP_DELAY_US = 3000,             // POR startup time
    SOFT_RESET_DELAY_US = 24000,         // wait after CMD soft reset
    SUSPEND_TO_NORMAL_DELAY_US = 38000,  // PMU suspend -> normal
    GOTO_SUSPEND_DELAY_US = 6000,        // PMU transition into suspend
    UPD_OAE_DELAY_US = 1000,             // wait after PMU_UPD_OAE
    OTP_POLL_DELAY_US = 300,             // OTP status polling interval
    BR_DELAY_US = 14000,                 // wait after PMU BR command
    FGR_DELAY_US = 18000,                // wait after PMU FGR command
    SELF_TEST_SUSPEND_DELAY_US = 30000,  // self-test entry suspend wait
    SELF_TEST_FGR_DELAY_US = 30000,      // self-test FGR settle
    SELF_TEST_BR_FAST_DELAY_US = 4000,   // self-test BR_FAST settle
    SELF_TEST_FM_FAST_DELAY_US = 16000,  // self-test FM_FAST settle
    SELF_TEST_CFG_DELAY_US = 1000,       // wait after self-test config write
    SELF_TEST_READ_DELAY_US = 6000,      // wait before self-test data read
  };

  enum DataLength : uint8_t
  {
    OTP_DATA_LENGTH = 32,
    MAG_TEMP_DATA_LEN = 12,
  };

  enum OtpIndex : uint8_t
  {
    OTP_TEMP_OFF_SENS = 0x0D,
    OTP_MAG_OFFSET_X = 0x0E,
    OTP_MAG_OFFSET_Y = 0x0F,
    OTP_MAG_OFFSET_Z = 0x10,
    OTP_MAG_SENS_X = 0x10,
    OTP_MAG_SENS_Y = 0x11,
    OTP_MAG_SENS_Z = 0x11,
    OTP_MAG_TCO_X = 0x12,
    OTP_MAG_TCO_Y = 0x13,
    OTP_MAG_TCO_Z = 0x14,
    OTP_MAG_TCS_X = 0x12,
    OTP_MAG_TCS_Y = 0x13,
    OTP_MAG_TCS_Z = 0x14,
    OTP_MAG_DUT_T0 = 0x18,
    OTP_CROSS_X_Y = 0x15,
    OTP_CROSS_Y_X = 0x15,
    OTP_CROSS_Z_X = 0x16,
    OTP_CROSS_Z_Y = 0x16,
  };

  enum SelfTestCmd : uint8_t
  {
    SELF_TEST_DISABLE = 0x00,
    SELF_TEST_POS_X = 0x0D,
    SELF_TEST_NEG_X = 0x0B,
    SELF_TEST_POS_Y = 0x15,
    SELF_TEST_NEG_Y = 0x13,
  };

  struct RawMagData
  {
    int32_t x = {};
    int32_t y = {};
    int32_t z = {};
    int32_t t = {};
  };

  struct MagCompensation
  {
    float offset_x = {};
    float offset_y = {};
    float offset_z = {};
    float t_offs = {};

    float sens_x = {};
    float sens_y = {};
    float sens_z = {};
    float t_sens = {};

    float tco_x = {};
    float tco_y = {};
    float tco_z = {};

    float tcs_x = {};
    float tcs_y = {};
    float tcs_z = {};

    float t0 = 23.0f;

    float cross_x_y = {};
    float cross_y_x = {};
    float cross_z_x = {};
    float cross_z_y = {};
  };

  struct PmuCmdStatus0
  {
    uint8_t pmu_cmd_busy = {};
    uint8_t odr_ovwr = {};
    uint8_t avr_ovwr = {};
    uint8_t pwr_mode_is_normal = {};
    uint8_t cmd_is_illegal = {};
    uint8_t pmu_cmd_value = {};
  };

  bool enterSuspendMode();
  bool suspendToNormalMode();
  bool setPowerMode(uint8_t _mode);
  bool checkWhoAmI();
  bool execSelfTest();
  bool applyConfiguration();
  static bool isValidOdr(uint8_t _odr);
  static bool isValidAveraging(uint8_t _averaging);
  bool configure();
  bool magneticResetAndWait();
  bool getPmuCmdStatus0(PmuCmdStatus0& _status);
  // bool selfTestEntryConfig();
  // bool selfTestConfig(uint8_t _st_cmd, float& _x_ut, float& _y_ut);
  /** @brief OTPレジスタを全件読み出して内部補償係数に反映する. */
  bool readOtpRegisters();
  /** @brief 指定したOTPワードを1件読み出す. */
  bool readOtpWord(uint8_t _addr, uint16_t& _word);
  /** @brief OTP生データから補償パラメータを更新する. */
  void updateCompensationFromOtp();
  bool setOdrPerformance(uint8_t _odr = ODR_100Hz, uint8_t _avg = AVG_4);
  bool readRawMagData(RawMagData& _raw);
  /**
   * @brief bmm350はレジスタ読み取り時の最初の２バイトが0x00で返ってくるため，必ず＋２バイト分読み取って最初の２バイトを捨てる
   * @note Datasheet 9.2.3 Dummy bites in I2C mode
   * Datasheet: https://www.bosch-sensortec.com/products/motion-sensors/magnetometers/bmm350/
   */
  bool readBytesWithDummy(uint8_t reg_addr, size_t length, uint8_t* rx);

  /**
   * @brief 補償済みX軸磁気データ [uT] を算出する.
   *
   * @param x X軸の補償前データ [uT]
   * @param y Y軸の補償前データ [uT]
   * @param z Z軸の補償前データ [uT]
   * @param temperature 温度データ [degC]
   * @return float 補償後X軸磁気データ [uT]
   */
  float compensateX(float _x, float _y, float _z, float _temperature) const;

  /**
   * @brief 補償済みY軸磁気データ [uT] を算出する.
   *
   * @param x X軸の補償前データ [uT]
   * @param y Y軸の補償前データ [uT]
   * @param z Z軸の補償前データ [uT]
   * @param temperature 温度データ [degC]
   * @return float 補償後Y軸磁気データ [uT]
   */
  float compensateY(float _x, float _y, float _z, float _temperature) const;

  /**
   * @brief 補償済みZ軸磁気データ [uT] を算出する.
   *
   * @param x X軸の補償前データ [uT]
   * @param y Y軸の補償前データ [uT]
   * @param z Z軸の補償前データ [uT]
   * @param temperature 温度データ [degC]
   * @return float 補償後Z軸磁気データ [uT]
   */
  float compensateZ(float _x, float _y, float _z, float _temperature) const;

  /** @brief nビット符号付き値として符号拡張する. */
  static int32_t fixSign(uint32_t _raw, int _bits);
  /** @brief マイクロ秒単位で待機する. */
  static void delayUs(uint32_t _period_us);

  linux::I2Cdev i2c_;
  uint16_t otp_data_[32]{};
  MagCompensation mag_comp_{};
  uint8_t axis_en_ = {};
};
}  // namespace driver
}  // namespace tobas
