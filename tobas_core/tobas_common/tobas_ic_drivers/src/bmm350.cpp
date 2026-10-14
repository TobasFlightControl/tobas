// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/bmm350.hpp"

#include <bitset>
#include <chrono>
#include <iostream>
#include <thread>

namespace tobas
{
namespace driver
{
BMM350::BMM350()
{
}

bool BMM350::initialize(const char* _i2c_device)
{
  if (!i2c_.initialize(_i2c_device, 0x14)) {
    std::cerr << "Failed to initialize I2C device.\n";
    return false;
  }

  delayUs(STARTUP_DELAY_US);

  if (!checkWhoAmI()) {
    return false;
  }

  if (!enterSuspendMode()) {
    return false;
  }

  if (!readOtpRegisters()) {
    return false;
  }

  if (!magneticResetAndWait()) {
    return false;
  }

  if (!applyConfiguration()) {
    return false;
  }

  if (!suspendToNormalMode()) {
    return false;
  }

  return true;
}

bool BMM350::readBytesWithDummy(uint8_t _reg_addr, size_t _length, uint8_t* _rx)
{
  constexpr size_t kDummyBytes = 2;
  uint8_t raw[MAG_TEMP_DATA_LEN + kDummyBytes] = {};

  if (!i2c_.readBytes(_reg_addr, _length + kDummyBytes, raw)) {
    return false;
  }

  for (size_t i = 0; i < _length; ++i) {
    _rx[i] = raw[i + kDummyBytes];
  }

  return true;
}

bool BMM350::readOtpWord(uint8_t _addr, uint16_t& _word)
{
  const uint8_t otp_cmd = static_cast<uint8_t>(OTP_DIR_READ | (_addr & OTP_WORD_ADDR_MSK));
  if (!i2c_.writeByte(OTP_CMD_REG, otp_cmd, false)) {
    return false;
  }

  uint8_t otp_status = 0;
  bool done = false;
  for (int i = 0; i < 100; ++i) {
    delayUs(OTP_POLL_DELAY_US);
    if (!readBytesWithDummy(OTP_STATUS_REG, 1, &otp_status)) {
      return false;
    }

    if ((otp_status & OTP_STATUS_ERROR_MSK) != 0) {
      return false;
    }

    if ((otp_status & OTP_STATUS_CMD_DONE) != 0) {
      done = true;
      break;
    }
  }

  if (!done) {
    std::cerr << "OTP read Timeout\n";
    return false;
  }

  uint8_t msb = 0;
  uint8_t lsb = 0;
  if (!readBytesWithDummy(OTP_DATA_MSB_REG, 1, &msb) || !readBytesWithDummy(OTP_DATA_LSB_REG, 1, &lsb)) {
    return false;
  }

  _word = static_cast<uint16_t>((static_cast<uint16_t>(msb) << 8) | lsb);
  return true;
}

bool BMM350::readOtpRegisters()
{
  for (uint8_t i = 0; i < OTP_DATA_LENGTH; ++i) {
    uint16_t word = 0;
    if (!readOtpWord(i, word)) {
      std::cerr << "Failed to read OTP word.\n";
      return false;
    }
    otp_data_[i] = word;
  }

  if (!i2c_.writeByte(OTP_CMD_REG, OTP_PWR_OFF, false)) {
    std::cerr << "Failed to power off OTP.\n";
    return false;
  }

  updateCompensationFromOtp();
  return true;
}

void BMM350::updateCompensationFromOtp()
{
  const uint16_t off_x = otp_data_[OTP_MAG_OFFSET_X] & 0x0FFF;
  const uint16_t off_y =
    static_cast<uint16_t>(((otp_data_[OTP_MAG_OFFSET_X] & 0xF000) >> 4) + (otp_data_[OTP_MAG_OFFSET_Y] & LSB_MASK));
  const uint16_t off_z =
    static_cast<uint16_t>((otp_data_[OTP_MAG_OFFSET_Y] & 0x0F00) + (otp_data_[OTP_MAG_OFFSET_Z] & LSB_MASK));
  const uint16_t t_off = otp_data_[OTP_TEMP_OFF_SENS] & LSB_MASK;

  mag_comp_.offset_x = static_cast<float>(fixSign(off_x, SIGNED_12_BIT));
  mag_comp_.offset_y = static_cast<float>(fixSign(off_y, SIGNED_12_BIT));
  mag_comp_.offset_z = static_cast<float>(fixSign(off_z, SIGNED_12_BIT));
  mag_comp_.t_offs = static_cast<float>(fixSign(t_off, SIGNED_8_BIT)) / 5.0f;

  const uint8_t sens_x = static_cast<uint8_t>((otp_data_[OTP_MAG_SENS_X] & MSB_MASK) >> 8);
  const uint8_t sens_y = static_cast<uint8_t>(otp_data_[OTP_MAG_SENS_Y] & LSB_MASK);
  const uint8_t sens_z = static_cast<uint8_t>((otp_data_[OTP_MAG_SENS_Z] & MSB_MASK) >> 8);
  const uint8_t t_sens = static_cast<uint8_t>((otp_data_[OTP_TEMP_OFF_SENS] & MSB_MASK) >> 8);

  mag_comp_.sens_x = static_cast<float>(fixSign(sens_x, SIGNED_8_BIT)) / 256.0f;
  mag_comp_.sens_y = static_cast<float>(fixSign(sens_y, SIGNED_8_BIT)) / 256.0f;
  mag_comp_.sens_z = static_cast<float>(fixSign(sens_z, SIGNED_8_BIT)) / 256.0f;
  mag_comp_.t_sens = static_cast<float>(fixSign(t_sens, SIGNED_8_BIT)) / 512.0f;

  const uint8_t tco_x = static_cast<uint8_t>(otp_data_[OTP_MAG_TCO_X] & LSB_MASK);
  const uint8_t tco_y = static_cast<uint8_t>(otp_data_[OTP_MAG_TCO_Y] & LSB_MASK);
  const uint8_t tco_z = static_cast<uint8_t>(otp_data_[OTP_MAG_TCO_Z] & LSB_MASK);

  mag_comp_.tco_x = static_cast<float>(fixSign(tco_x, SIGNED_8_BIT)) / 32.0f;
  mag_comp_.tco_y = static_cast<float>(fixSign(tco_y, SIGNED_8_BIT)) / 32.0f;
  mag_comp_.tco_z = static_cast<float>(fixSign(tco_z, SIGNED_8_BIT)) / 32.0f;

  const uint8_t tcs_x = static_cast<uint8_t>((otp_data_[OTP_MAG_TCS_X] & MSB_MASK) >> 8);
  const uint8_t tcs_y = static_cast<uint8_t>((otp_data_[OTP_MAG_TCS_Y] & MSB_MASK) >> 8);
  const uint8_t tcs_z = static_cast<uint8_t>((otp_data_[OTP_MAG_TCS_Z] & MSB_MASK) >> 8);

  mag_comp_.tcs_x = static_cast<float>(fixSign(tcs_x, SIGNED_8_BIT)) / 16384.0f;
  mag_comp_.tcs_y = static_cast<float>(fixSign(tcs_y, SIGNED_8_BIT)) / 16384.0f;
  mag_comp_.tcs_z = static_cast<float>(fixSign(tcs_z, SIGNED_8_BIT)) / 16384.0f;

  mag_comp_.t0 = static_cast<float>(fixSign(otp_data_[OTP_MAG_DUT_T0], SIGNED_16_BIT)) / 512.0f + 23.0f;

  const uint8_t cross_x_y = static_cast<uint8_t>(otp_data_[OTP_CROSS_X_Y] & LSB_MASK);
  const uint8_t cross_y_x = static_cast<uint8_t>((otp_data_[OTP_CROSS_Y_X] & MSB_MASK) >> 8);
  const uint8_t cross_z_x = static_cast<uint8_t>(otp_data_[OTP_CROSS_Z_X] & LSB_MASK);
  const uint8_t cross_z_y = static_cast<uint8_t>((otp_data_[OTP_CROSS_Z_Y] & MSB_MASK) >> 8);

  mag_comp_.cross_x_y = static_cast<float>(fixSign(cross_x_y, SIGNED_8_BIT)) / 800.0f;
  mag_comp_.cross_y_x = static_cast<float>(fixSign(cross_y_x, SIGNED_8_BIT)) / 800.0f;
  mag_comp_.cross_z_x = static_cast<float>(fixSign(cross_z_x, SIGNED_8_BIT)) / 800.0f;
  mag_comp_.cross_z_y = static_cast<float>(fixSign(cross_z_y, SIGNED_8_BIT)) / 800.0f;
}

void BMM350::configure(ODR _odr, Averaging _averaging)
{
  odr_ = _odr;
  averaging_ = _averaging;
}

bool BMM350::applyConfiguration()
{
  if (!i2c_.writeByte(PAD_CTRL_REG, PAD_DRIVE_STRONGEST, false)) {
    std::cerr << "Failed to configure pad drive.\n";
    return false;
  }

  if (!setOdrPerformance()) {
    std::cerr << "Failed to set ODR.\n";
    return false;
  }

  axis_en_ = ENABLE_XYZ;
  if (!i2c_.writeByte(PMU_CMD_AXIS_EN_REG, axis_en_, false)) {
    std::cerr << "Failed to enable all axes.\n";
    return false;
  }

  return true;
}

bool BMM350::isValidOdr(uint8_t _odr)
{
  return _odr == ODR_100Hz || _odr == ODR_25Hz;
}

bool BMM350::isValidAveraging(uint8_t _averaging)
{
  return _averaging == AVG_2 || _averaging == AVG_4;
}

bool BMM350::setOdrPerformance(uint8_t _odr, uint8_t _avg)
{
  uint8_t reg;
  reg = static_cast<uint8_t>((_odr & ~AVG_MSK) | ((_avg << AVG_POS) & AVG_MSK));

  if (!i2c_.writeByte(PMU_CMD_AGGR_SET_REG, reg, false)) {
    std::cerr << "Failed to configure ODR/performance.\n";
    return false;
  }

  if (!i2c_.writeByte(PMU_CMD_REG, PMU_UPD_OAE, false)) {
    std::cerr << "Failed to apply ODR/performance.\n";
    return false;
  }
  // Wait for the setting to take effect with a fixed delay matching the Bosch SensorAPI.
  delayUs(UPD_OAE_DELAY_US);
  return true;
}

bool BMM350::enterSuspendMode()
{
  if (!i2c_.writeByte(CMD_REG, SOFT_RESET_CMD, false)) {
    std::cerr << "Failed to issue soft-reset.\n";
    return false;
  }
  delayUs(SOFT_RESET_DELAY_US);

  if (!i2c_.writeByte(PMU_CMD_REG, PMU_SUSPEND, false)) {
    std::cerr << "Failed to enter suspend mode.\n";
    return false;
  }
  delayUs(GOTO_SUSPEND_DELAY_US);
  return true;
}

bool BMM350::suspendToNormalMode()
{
  return setPowerMode(PMU_NORMAL);
}

bool BMM350::setPowerMode(uint8_t _mode)
{
  uint8_t last_mode;
  if (!readBytesWithDummy(PMU_CMD_REG, 1, &last_mode)) {
    std::cerr << "Failed to read PMU command.\n";
    return false;
  }

  if (last_mode == PMU_NORMAL || last_mode == PMU_UPD_OAE) {
    if (!i2c_.writeByte(PMU_CMD_REG, PMU_SUSPEND, false)) {
      std::cerr << "Failed to transit PMU to suspend.\n";
      return false;
    }
    // Wait for the suspend transition to complete with a fixed delay compliant with SensorAPI.
    delayUs(GOTO_SUSPEND_DELAY_US);
  }

  if (!i2c_.writeByte(PMU_CMD_REG, _mode, false)) {
    std::cerr << "Failed to set PMU mode.\n";
    return false;
  }

  if (_mode == PMU_NORMAL) {
    // Specified wait time required for the suspend-to-normal transition.
    delayUs(SUSPEND_TO_NORMAL_DELAY_US);
  }
  return true;
}

bool BMM350::checkWhoAmI()
{
  uint8_t chip_id = 0;
  if (!readBytesWithDummy(WHO_AM_I_REG, 1, &chip_id)) {
    std::cerr << "Failed to read chip ID.\n";
    return false;
  }
  if (chip_id != CHIP_ID) {
    std::cerr << "Who-Am-I check failed.\n";
    return false;
  }
  return true;
}

bool BMM350::getPmuCmdStatus0(PmuCmdStatus0& _status)
{
  uint8_t reg = 0;
  if (!readBytesWithDummy(PMU_CMD_STATUS_0_REG, 1, &reg)) {
    return false;
  }

  _status.pmu_cmd_busy = reg & PMU_CMD_BUSY_MSK;
  _status.odr_ovwr = (reg & ODR_OVWR_MSK) >> ODR_OVWR_POS;
  _status.avr_ovwr = (reg & AVG_OVWR_MSK) >> AVG_OVWR_POS;
  _status.pwr_mode_is_normal = (reg & PWR_MODE_IS_NORMAL_MSK) >> PWR_MODE_IS_NORMAL_POS;
  _status.cmd_is_illegal = (reg & CMD_IS_ILLEGAL_MSK) >> CMD_IS_ILLEGAL_POS;
  _status.pmu_cmd_value = (reg & PMU_CMD_VALUE_MSK) >> PMU_CMD_VALUE_POS;
  return true;
}

bool BMM350::magneticResetAndWait()
{
  PmuCmdStatus0 status{};
  bool restore_normal = false;

  if (!getPmuCmdStatus0(status)) {
    std::cerr << "Failed to read PMU status before magnetic reset.\n";
    return false;
  }

  if (status.pwr_mode_is_normal != 0) {
    restore_normal = true;
    if (!setPowerMode(PMU_SUSPEND)) {
      return false;
    }
  }

  if (!i2c_.writeByte(PMU_CMD_REG, PMU_BR, false)) {
    std::cerr << "Failed to issue BR.\n";
    return false;
  }
  delayUs(BR_DELAY_US);

  if (!getPmuCmdStatus0(status)) {
    return false;
  }
  if (status.pmu_cmd_value != PMU_BR) {
    std::cerr << "BR command did not complete as expected.\n";
    return false;
  }

  if (!i2c_.writeByte(PMU_CMD_REG, PMU_FGR, false)) {
    std::cerr << "Failed to issue FGR.\n";
    return false;
  }
  delayUs(FGR_DELAY_US);

  if (!getPmuCmdStatus0(status)) {
    return false;
  }
  if (status.pmu_cmd_value != PMU_FGR) {
    std::cerr << "FGR command did not complete as expected.\n";
    return false;
  }

  if (restore_normal && !setPowerMode(PMU_NORMAL)) {
    return false;
  }
  return true;
}

bool BMM350::readRawMagData(RawMagData& _raw)
{
  if (!readBytesWithDummy(OUT_X_XLSB_REG, MAG_TEMP_DATA_LEN, raw_data_)) {
    return false;
  }

  const uint32_t raw_x = static_cast<uint32_t>(raw_data_[0]) | (static_cast<uint32_t>(raw_data_[1]) << 8) |
                         (static_cast<uint32_t>(raw_data_[2]) << 16);
  const uint32_t raw_y = static_cast<uint32_t>(raw_data_[3]) | (static_cast<uint32_t>(raw_data_[4]) << 8) |
                         (static_cast<uint32_t>(raw_data_[5]) << 16);
  const uint32_t raw_z = static_cast<uint32_t>(raw_data_[6]) | (static_cast<uint32_t>(raw_data_[7]) << 8) |
                         (static_cast<uint32_t>(raw_data_[8]) << 16);
  const uint32_t raw_t = static_cast<uint32_t>(raw_data_[9]) | (static_cast<uint32_t>(raw_data_[10]) << 8) |
                         (static_cast<uint32_t>(raw_data_[11]) << 16);

  _raw.x = ((axis_en_ & 0x01) == 0) ? 0 : fixSign(raw_x, SIGNED_24_BIT);
  _raw.y = ((axis_en_ & 0x02) == 0) ? 0 : fixSign(raw_y, SIGNED_24_BIT);
  _raw.z = ((axis_en_ & 0x04) == 0) ? 0 : fixSign(raw_z, SIGNED_24_BIT);
  _raw.t = fixSign(raw_t, SIGNED_24_BIT);
  return true;
}

float BMM350::compensateX(float _x, float _y, float, float) const
{
  const float denom = 1.0f - (mag_comp_.cross_y_x * mag_comp_.cross_x_y);
  return (_x - mag_comp_.cross_x_y * _y) / denom;
}

float BMM350::compensateY(float _x, float _y, float, float) const
{
  const float denom = 1.0f - (mag_comp_.cross_y_x * mag_comp_.cross_x_y);
  return (_y - mag_comp_.cross_y_x * _x) / denom;
}

float BMM350::compensateZ(float _x, float _y, float _z, float) const
{
  const float denom = 1.0f - (mag_comp_.cross_y_x * mag_comp_.cross_x_y);
  return _z + (_x * (mag_comp_.cross_y_x * mag_comp_.cross_z_y - mag_comp_.cross_z_x) -
               _y * (mag_comp_.cross_z_y - mag_comp_.cross_x_y * mag_comp_.cross_z_x)) /
                denom;
}

bool BMM350::readMag(float& _mx, float& _my, float& _mz)
{
  if (!readRawMagData(raw_)) {
    std::cerr << "Failed to read magnetic field.\n";
    return false;
  }

  constexpr float bxy_sens = 14.55f;
  constexpr float bz_sens = 9.0f;
  constexpr float temp_sens = 0.00204f;
  constexpr float ina_xy_gain_trgt = 19.46f;
  constexpr float ina_z_gain_trgt = 31.0f;
  constexpr float adc_gain = 1.0f / 1.5f;
  constexpr float lut_gain = 0.714607238769531f;
  constexpr float power = 1000000.0f / 1048576.0f;

  constexpr float lsb_to_ut_x = power / (bxy_sens * ina_xy_gain_trgt * adc_gain * lut_gain);
  constexpr float lsb_to_ut_y = lsb_to_ut_x;
  constexpr float lsb_to_ut_z = power / (bz_sens * ina_z_gain_trgt * adc_gain * lut_gain);
  constexpr float lsb_to_degc_t = 1.0f / (temp_sens * adc_gain * lut_gain * 1048576.0f);

  float out_x = static_cast<float>(raw_.x) * lsb_to_ut_x;
  float out_y = static_cast<float>(raw_.y) * lsb_to_ut_y;
  float out_z = static_cast<float>(raw_.z) * lsb_to_ut_z;
  float out_t = static_cast<float>(raw_.t) * lsb_to_degc_t - 25.49f;

  out_t = (1.0f + mag_comp_.t_sens) * out_t + mag_comp_.t_offs;

  float comp[3] = { out_x, out_y, out_z };
  const float offset[3] = { mag_comp_.offset_x, mag_comp_.offset_y, mag_comp_.offset_z };
  const float sens[3] = { mag_comp_.sens_x, mag_comp_.sens_y, mag_comp_.sens_z };
  const float tco[3] = { mag_comp_.tco_x, mag_comp_.tco_y, mag_comp_.tco_z };
  const float tcs[3] = { mag_comp_.tcs_x, mag_comp_.tcs_y, mag_comp_.tcs_z };

  for (int i = 0; i < 3; ++i) {
    comp[i] *= 1.0f + sens[i];
    comp[i] += offset[i];
    comp[i] += tco[i] * (out_t - mag_comp_.t0);
    comp[i] /= 1.0f + tcs[i] * (out_t - mag_comp_.t0);
  }

  const float cr_x = compensateX(comp[0], comp[1], comp[2], out_t);
  const float cr_y = compensateY(comp[0], comp[1], comp[2], out_t);
  const float cr_z = compensateZ(comp[0], comp[1], comp[2], out_t);

  const float x_ut = ((axis_en_ & 0x01) == 0) ? 0.0f : cr_x;
  const float y_ut = ((axis_en_ & 0x02) == 0) ? 0.0f : cr_y;
  const float z_ut = ((axis_en_ & 0x04) == 0) ? 0.0f : cr_z;
  _mx = x_ut;
  _my = y_ut;
  _mz = z_ut;
  return true;
}

int32_t BMM350::fixSign(uint32_t _raw, int _bits)
{
  int32_t sign_bit_index = 0;
  int32_t full_scale = 0;
  switch (_bits) {
    case SIGNED_8_BIT:
      sign_bit_index = 7;
      full_scale = 1 << 8;
      break;
    case SIGNED_12_BIT:
      sign_bit_index = 11;
      full_scale = 1 << 12;
      break;
    case SIGNED_16_BIT:
      sign_bit_index = 15;
      full_scale = 1 << 16;
      break;
    case SIGNED_24_BIT:
      sign_bit_index = 23;
      full_scale = 1 << 24;
      break;
    default:
      return static_cast<int32_t>(_raw);
  }

  const std::bitset<32> raw_bits(_raw);
  int32_t value = static_cast<int32_t>(_raw);
  if (raw_bits.test(static_cast<size_t>(sign_bit_index))) {
    value -= full_scale;
  }
  return value;
}

void BMM350::delayUs(uint32_t _period_us)
{
  std::this_thread::sleep_for(std::chrono::microseconds(_period_us));
}
}  // namespace driver
}  // namespace tobas
