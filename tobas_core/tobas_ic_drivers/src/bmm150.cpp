#include "tobas_ic_drivers/bmm150.hpp"

#include <bitset>
#include <cstring>
#include <iostream>
#include <thread>

using namespace std;

namespace driver
{
BMM150::BMM150()
{
}

bool BMM150::initialize()
{
  if (!i2c_.initialize(kI2cDevice, kI2cAddress)) {
    cerr << "Failed to initialize I2C device." << endl;
    return false;
  }

  if (!enterSuspendMode()) {
    cerr << "Failed to enter suspend mode." << endl;
    return false;
  }

  if (!suspendToSleepMode()) {
    cerr << "Failed to change mode from suspend to sleep." << endl;
    return false;
  }

  if (!checkWhoAmI()) {
    cerr << "Who-Am-I check failed." << endl;
    return false;
  }

  if (!execSelfTest()) {
    cerr << "self test failed." << endl;
    return false;
  }

  if (!readTrimRegisters()) {
    cerr << "Failed to read trim registers." << endl;
    return false;
  }

  if (!configure()) {
    cerr << "Failed to configure magnetometer." << endl;
    return false;
  }

  return true;
}

bool BMM150::readMag(double& mx, double& my, double& mz)
{
  if (!i2c_.readBytes(OUTX_LSB_REG, 8, rx_)) {
    return false;
  }

  if ((rx_[0] & 1) != kSelfTestRef) {
    cerr << "Failed in x-axis self test." << endl;
    return false;
  }
  if ((rx_[2] & 1) != kSelfTestRef) {
    cerr << "Failed in y-axis self test." << endl;
    return false;
  }
  if ((rx_[4] & 1) != kSelfTestRef) {
    cerr << "Failed in z-axis self test." << endl;
    return false;
  }
  // if (((rx_[6] & 1) == 0)) {
  //   cerr << "Read Old Data" << endl;
  //   return false;
  // }

  const int16_t msb_data_x = static_cast<int16_t>(static_cast<int8_t>(rx_[1]) * 32);
  const int16_t msb_data_y = static_cast<int16_t>(static_cast<int8_t>(rx_[3]) * 32);
  const int16_t msb_data_z = static_cast<int16_t>(static_cast<int8_t>(rx_[5]) * 128);
  const int16_t raw_data_x = static_cast<int16_t>(msb_data_x | (rx_[0] >> 3));
  const int16_t raw_data_y = static_cast<int16_t>(msb_data_y | (rx_[2] >> 3));
  const int16_t raw_data_z = static_cast<int16_t>(msb_data_z | (rx_[4] >> 1));
  const uint16_t r_hall = ((static_cast<uint16_t>(rx_[7]) << 6) | (rx_[6] >> 2));

  // Compensate for the temperature effect using resistance value of hall sensor based on
  // https://github.com/boschsensortec/BMM150_SensorAPI/blob/master/bmm150.c compensate_x
  mx = static_cast<double>(compensateX(raw_data_x, r_hall)) * kResolution;
  my = static_cast<double>(compensateY(raw_data_y, r_hall)) * kResolution;
  mz = static_cast<double>(compensateZ(raw_data_z, r_hall)) * kResolution;

  return true;
}

bool BMM150::enterSuspendMode()
{
  if (!i2c_.writeByte(CFG_REG_A, 0x00, true)) {
    cerr << "Failed to write power control reg." << endl;
    return false;
  }

  this_thread::sleep_for(3ms);  // Wait for BMM150's suspend.
  return true;
}

bool BMM150::suspendToSleepMode()
{
  if (!i2c_.writeByte(CFG_REG_A, 0x01), true) {
    cerr << "Failed to write power control reg." << endl;
    return false;
  }

  this_thread::sleep_for(3ms);  // Wait for BMM150's start-up.
  return true;
}

bool BMM150::checkWhoAmI()
{
  uint8_t byte;

  if (!i2c_.readByte(WHO_AM_I_REG, byte)) {
    cerr << "Failed to read WHO_AM_I data." << endl;
    return false;
  }

  if (byte != CHIP_ID) {
    cerr << "Magnetometer is not recognized." << endl;
    return false;
  }

  return true;
}

bool BMM150::execSelfTest()
{
  if (!i2c_.writeByte(CFG_REG_B, ADV_SELF_TEST_NORMAL | ODR_10HZ | OP_SLEEP | SELF_TEST), true) {
    cerr << "Failed to write to CFG_REG_B." << endl;
    return false;
  }

  this_thread::sleep_for(3ms);  // Wait for BMM150's self test

  return true;
}

bool BMM150::configure()
{
  if (!i2c_.writeByte(CFG_REG_B, ADV_SELF_TEST_NORMAL | ODR_30HZ | OP_NORMAL), true) {
    cerr << "Failed to write to CFG_REG_B." << endl;
    return false;
  }

  if (!i2c_.writeByte(CFG_REG_E, REPXY), true) {
    cerr << "Failed to write to CFG_REG_E." << endl;
    return false;
  }

  if (!i2c_.writeByte(CFG_REG_F, REPZ), true) {
    cerr << "Failed to write to CFG_REG_F." << endl;
    return false;
  }

  return true;
}

bool BMM150::readTrimRegisters()
{
  uint8_t trim_x1y1[2] = { 0 };
  uint8_t trim_xyz_data[4] = { 0 };
  uint8_t trim_xy1xy2[10] = { 0 };
  uint16_t temp_msb = 0;

  // Trim register value is read
  if (!i2c_.readBytes(DIG_X1_REG, 2, rx_)) {
    cerr << "Failed to read DIG_X1_REG." << endl;
    return false;
  }
  memcpy(trim_x1y1, rx_, 2);
  if (!i2c_.readBytes(DIG_Z4_LSB_REG, 4, rx_)) {
    cerr << "Failed to read DIG_Z4_REG." << endl;
    return false;
  }
  memcpy(trim_xyz_data, rx_, 4);
  if (!i2c_.readBytes(DIG_Z2_LSB_REG, 10, rx_)) {
    cerr << "Failed to read DIG_Z2_LSB_REG." << endl;
    return false;
  }
  memcpy(trim_xy1xy2, rx_, 10);

  // Trim data which is read is updated in the device structure
  trim_data.dig_x1 = static_cast<int8_t>(trim_x1y1[0]);
  trim_data.dig_y1 = static_cast<int8_t>(trim_x1y1[1]);
  trim_data.dig_x2 = static_cast<int8_t>(trim_xyz_data[2]);
  trim_data.dig_y2 = static_cast<int8_t>(trim_xyz_data[3]);
  temp_msb = (static_cast<uint16_t>(trim_xy1xy2[3])) << 8;
  trim_data.dig_z1 = static_cast<uint16_t>(temp_msb | trim_xy1xy2[2]);
  temp_msb = (static_cast<uint16_t>(trim_xy1xy2[1])) << 8;
  trim_data.dig_z2 = static_cast<int16_t>(temp_msb | trim_xy1xy2[0]);
  temp_msb = (static_cast<uint16_t>(trim_xy1xy2[7])) << 8;
  trim_data.dig_z3 = static_cast<int16_t>(temp_msb | trim_xy1xy2[6]);
  temp_msb = (static_cast<uint16_t>(trim_xyz_data[1])) << 8;
  trim_data.dig_z4 = static_cast<int16_t>(temp_msb | trim_xyz_data[0]);
  trim_data.dig_xy1 = trim_xy1xy2[9];
  trim_data.dig_xy2 = static_cast<int8_t>(trim_xy1xy2[8]);
  temp_msb = (static_cast<uint16_t>(trim_xy1xy2[5] & 0x7F)) << 8;
  trim_data.dig_xyz1 = static_cast<uint16_t>(temp_msb | trim_xy1xy2[4]);

  return true;
}

int16_t BMM150::compensateX(const int16_t& mag_data_x, const uint16_t& data_r_hall)
{
  uint16_t process_comp_x0 = 0;

  // Overflow condition check
  if (mag_data_x != kXyaxesFlipOverflowAdcval) {
    if (data_r_hall != 0) {
      // Availability of valid data
      process_comp_x0 = data_r_hall;
    }
    else if (trim_data.dig_xyz1 != 0) {
      process_comp_x0 = trim_data.dig_xyz1;
    }
    else {
      process_comp_x0 = 0;
    }
    if (process_comp_x0 != 0) {
      // Processing compensation equations
      const int32_t process_comp_x1 = ((int32_t)trim_data.dig_xyz1) * 16384;
      const uint16_t process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
      int16_t retval = ((int16_t)process_comp_x2);
      const int32_t process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
      const int32_t process_comp_x4 = (((int32_t)trim_data.dig_xy2) * (process_comp_x3 / 128));
      const int32_t process_comp_x5 = (int32_t)(((int16_t)trim_data.dig_xy1) * 128);
      const int32_t process_comp_x6 = ((int32_t)retval) * process_comp_x5;
      const int32_t process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
      const int32_t process_comp_x8 = ((int32_t)(((int16_t)trim_data.dig_x2) + ((int16_t)0xA0)));
      const int32_t process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
      const int32_t process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
      retval = ((int16_t)(process_comp_x10 / 8192));
      return (retval + (((int16_t)trim_data.dig_x1) * 8)) / 16;
    }
    else {
      return kOverflowOutput;
    }
  }
  else {
    // Overflow condition
    return kOverflowOutput;
  }
}

int16_t BMM150::compensateY(const int16_t& mag_data_y, const uint16_t& data_r_hall)
{
  uint16_t process_comp_y0 = 0;

  // Overflow condition check
  if (mag_data_y != kXyaxesFlipOverflowAdcval) {
    if (data_r_hall != 0) {
      // Availability of valid data
      process_comp_y0 = data_r_hall;
    }
    else if (trim_data.dig_xyz1 != 0) {
      process_comp_y0 = trim_data.dig_xyz1;
    }
    else {
      process_comp_y0 = 0;
    }
    if (process_comp_y0 != 0) {
      // Processing compensation equations
      const int32_t process_comp_y1 = (((int32_t)trim_data.dig_xyz1) * 16384) / process_comp_y0;
      const uint16_t process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
      int16_t retval = ((int16_t)process_comp_y2);
      const int32_t process_comp_y3 = ((int32_t)retval) * ((int32_t)retval);
      const int32_t process_comp_y4 = ((int32_t)trim_data.dig_xy2) * (process_comp_y3 / 128);
      const int32_t process_comp_y5 = ((int32_t)(((int16_t)trim_data.dig_xy1) * 128));
      const int32_t process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
      const int32_t process_comp_y7 = ((int32_t)(((int16_t)trim_data.dig_y2) + ((int16_t)0xA0)));
      const int32_t process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
      const int32_t process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
      retval = (int16_t)(process_comp_y9 / 8192);
      return (retval + (((int16_t)trim_data.dig_y1) * 8)) / 16;
    }
    else {
      return kOverflowOutput;
    }
  }
  else {
    // Overflow condition
    return kOverflowOutput;
  }
}

int16_t BMM150::compensateZ(const int16_t& mag_data_z, const uint16_t& data_r_hall)
{
  if (mag_data_z != kZaxisHallOverflowAdcval) {
    if ((trim_data.dig_z2 != 0) && (trim_data.dig_z1 != 0) && (data_r_hall != 0) && (trim_data.dig_xyz1 != 0)) {
      // Processing compensation equations
      const int16_t process_comp_z0 = ((int16_t)data_r_hall) - ((int16_t)trim_data.dig_xyz1);
      const int32_t process_comp_z1 = (((int32_t)trim_data.dig_z3) * ((int32_t)(process_comp_z0))) / 4;
      const int32_t process_comp_z2 = (((int32_t)(mag_data_z - trim_data.dig_z4)) * 32768);
      const int32_t process_comp_z3 = ((int32_t)trim_data.dig_z1) * (((int16_t)data_r_hall) * 2);
      const int16_t process_comp_z4 = (int16_t)((process_comp_z3 + 32768) / 65536);
      const int32_t retval = ((process_comp_z2 - process_comp_z1) / (trim_data.dig_z2 + process_comp_z4));

      // Saturate result to +/- 2 micro-tesla
      if (retval > kPositiveSaturationZ) {
        return kPositiveSaturationZ;
      }
      else {
        if (retval < kNegativeSaturationZ) {
          return kNegativeSaturationZ;
        }
      }
      return static_cast<int16_t>(retval / 16);
    }
    else {
      return kOverflowOutput;
    }
  }
  else {
    // Overflow condition
    return kOverflowOutput;
  }
}
}  // namespace driver
