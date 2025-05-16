#include "tobas_ic_drivers/bmm150.hpp"

#include <iostream>
#include <bitset>
#include <thread>
#include <cstring>

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
  if (!readRegs(OUTX_LSB_REG, 8)) {
    return false;
  }

  if ((i2c_.rx[0] & 1) != SELF_TEST_X) {
    cerr << "Failed in x-axis self test." << endl;
    return false;
  }
  if ((i2c_.rx[2] & 1) != SELF_TEST_Y) {
    cerr << "Failed in y-axis self test." << endl;
    return false;
  }
  if ((i2c_.rx[4] & 1) != SELF_TEST_Z) {
    cerr << "Failed in z-axis self test." << endl;
    return false;
  }
  //   if (((i2c_.rx[6] & 1) == 0)){
  //     cerr << "Read Old Data" << endl;
  //     return false;
  //   }

  int16_t raw_data_x = ((i2c_.rx[1] << 5) | (i2c_.rx[0] >> 3));
  int16_t raw_data_y = ((i2c_.rx[3] << 5) | (i2c_.rx[2] >> 3));
  int16_t raw_data_z = ((i2c_.rx[5] << 7) | (i2c_.rx[4] >> 1));
  uint16_t r_hall = ((i2c_.rx[7] << 6) | (i2c_.rx[6] >> 2));

  // compensate for the temperature effect using resistance value of hall sensor based on
  // https://github.com/boschsensortec/BMM150_SensorAPI/blob/master/bmm150.c compensate_x
  mx = static_cast<double>(compensateX(raw_data_x, r_hall)) / 16.0;
  my = static_cast<double>(compensateY(raw_data_y, r_hall)) / 16.0;
  mz = static_cast<double>(compensateZ(raw_data_z, r_hall)) / 16.0;

  return true;
}

bool BMM150::writeReg(const uint8_t& addr, const uint8_t& data)
{
  i2c_.tx[0] = data;
  return i2c_.writeBytes(addr, 1);
}

bool BMM150::readRegs(const uint8_t& addr, const size_t& bytes)
{
  // ref: p.39 複数バイト読み込むときは自動的に読み出す対象のregisterのアドレスがincrementされる
  if (!i2c_.readBytes(addr, bytes)) {
    cerr << "Failed to read " << bytes << " bytes from " << bitset<8>(addr) << "." << endl;
    return false;
  }

  return true;
}

bool BMM150::checkWhoAmI()
{
  if (!readRegs(WHO_AM_I_REG, 1)) {
    cerr << "Failed to read WHO_AM_I data." << endl;
    return false;
  }

  if (i2c_.rx[0] != CHIP_ID) {
    cerr << "Magnetometer is not recognized." << endl;
    return false;
  }

  return true;
}

bool BMM150::execSelfTest()
{
  if (!writeReg(CFG_REG_B, ADV_SELF_TEST_NORMAL | ODR_10HZ | OP_SLEEP | SELF_TEST)) {
    cerr << "Failed to write to CFG_REG_B." << endl;
    return false;
  }

  this_thread::sleep_for(3ms);  // wait for BMM150's self test

  return true;
}

bool BMM150::configure()
{
  // XXX: サンプリング周波数が高いほどモータなど外部磁場の影響を受けやすくなる．おそらく電流値を下げるのが大事．
  if (!writeReg(CFG_REG_B, ADV_SELF_TEST_NORMAL | ODR_30HZ | OP_NORMAL)) {
    cerr << "Failed to write to CFG_REG_B." << endl;
    return false;
  }

  if (!writeReg(CFG_REG_E, REPXY)) {
    cerr << "Failed to write to CFG_REG_E." << endl;
    return false;
  }

  if (!writeReg(CFG_REG_F, REPZ)) {
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

  /* Trim register value is read */
  if (!readRegs(DIG_X1_REG, 2)) {
    cerr << "Failed to read DIG_X1_REG." << endl;
    return false;
  }
  memcpy(trim_x1y1, i2c_.rx, 2);
  if (!readRegs(DIG_Z4_LSB_REG, 4)) {
    cerr << "Failed to read DIG_Z4_REG." << endl;
    return false;
  }
  memcpy(trim_xyz_data, i2c_.rx, 4);
  if (!readRegs(DIG_Z2_LSB_REG, 10)) {
    cerr << "Failed to read DIG_Z2_LSB_REG." << endl;
    return false;
  }
  memcpy(trim_xy1xy2, i2c_.rx, 10);

  /*  Trim data which is read is updated
      in the device structure */
  trim_data.dig_x1 = (int8_t)trim_x1y1[0];
  trim_data.dig_y1 = (int8_t)trim_x1y1[1];
  trim_data.dig_x2 = (int8_t)trim_xyz_data[2];
  trim_data.dig_y2 = (int8_t)trim_xyz_data[3];
  temp_msb = ((uint16_t)trim_xy1xy2[3]) << 8;
  trim_data.dig_z1 = (uint16_t)(temp_msb | trim_xy1xy2[2]);
  temp_msb = ((uint16_t)trim_xy1xy2[1]) << 8;
  trim_data.dig_z2 = (int16_t)(temp_msb | trim_xy1xy2[0]);
  temp_msb = ((uint16_t)trim_xy1xy2[7]) << 8;
  trim_data.dig_z3 = (int16_t)(temp_msb | trim_xy1xy2[6]);
  temp_msb = ((uint16_t)trim_xyz_data[1]) << 8;
  trim_data.dig_z4 = (int16_t)(temp_msb | trim_xyz_data[0]);
  trim_data.dig_xy1 = trim_xy1xy2[9];
  trim_data.dig_xy2 = (int8_t)trim_xy1xy2[8];
  temp_msb = ((uint16_t)(trim_xy1xy2[5] & 0x7F)) << 8;
  trim_data.dig_xyz1 = (uint16_t)(temp_msb | trim_xy1xy2[4]);

  return true;
}

int16_t BMM150::compensateX(const int16_t& mag_data_x, const uint16_t& data_r_hall)
{
  int16_t retval;
  uint16_t process_comp_x0 = 0;
  int32_t process_comp_x1;
  uint16_t process_comp_x2;
  int32_t process_comp_x3;
  int32_t process_comp_x4;
  int32_t process_comp_x5;
  int32_t process_comp_x6;
  int32_t process_comp_x7;
  int32_t process_comp_x8;
  int32_t process_comp_x9;
  int32_t process_comp_x10;

  /* Overflow condition check */
  if (mag_data_x != XYAXES_FLIP_OVERFLOW_ADCVAL) {
    if (data_r_hall != 0) {
      /* Availability of valid data*/
      process_comp_x0 = data_r_hall;
    }
    else if (trim_data.dig_xyz1 != 0) {
      process_comp_x0 = trim_data.dig_xyz1;
    }
    else {
      process_comp_x0 = 0;
    }
    if (process_comp_x0 != 0) {
      /* Processing compensation equations*/
      process_comp_x1 = ((int32_t)trim_data.dig_xyz1) * 16384;
      process_comp_x2 = ((uint16_t)(process_comp_x1 / process_comp_x0)) - ((uint16_t)0x4000);
      retval = ((int16_t)process_comp_x2);
      process_comp_x3 = (((int32_t)retval) * ((int32_t)retval));
      process_comp_x4 = (((int32_t)trim_data.dig_xy2) * (process_comp_x3 / 128));
      process_comp_x5 = (int32_t)(((int16_t)trim_data.dig_xy1) * 128);
      process_comp_x6 = ((int32_t)retval) * process_comp_x5;
      process_comp_x7 = (((process_comp_x4 + process_comp_x6) / 512) + ((int32_t)0x100000));
      process_comp_x8 = ((int32_t)(((int16_t)trim_data.dig_x2) + ((int16_t)0xA0)));
      process_comp_x9 = ((process_comp_x7 * process_comp_x8) / 4096);
      process_comp_x10 = ((int32_t)mag_data_x) * process_comp_x9;
      retval = ((int16_t)(process_comp_x10 / 8192));
      retval = (retval + (((int16_t)trim_data.dig_x1) * 8)) / 16;
    }
    else {
      retval = OVERFLOW_OUTPUT;
    }
  }
  else {
    /* Overflow condition */
    retval = OVERFLOW_OUTPUT;
  }

  return retval;
}

int16_t BMM150::compensateY(const int16_t& mag_data_y, const uint16_t& data_r_hall)
{
  int16_t retval;
  uint16_t process_comp_y0 = 0;
  int32_t process_comp_y1;
  uint16_t process_comp_y2;
  int32_t process_comp_y3;
  int32_t process_comp_y4;
  int32_t process_comp_y5;
  int32_t process_comp_y6;
  int32_t process_comp_y7;
  int32_t process_comp_y8;
  int32_t process_comp_y9;

  /* Overflow condition check */
  if (mag_data_y != XYAXES_FLIP_OVERFLOW_ADCVAL) {
    if (data_r_hall != 0) {
      /* Availability of valid data*/
      process_comp_y0 = data_r_hall;
    }
    else if (trim_data.dig_xyz1 != 0) {
      process_comp_y0 = trim_data.dig_xyz1;
    }
    else {
      process_comp_y0 = 0;
    }
    if (process_comp_y0 != 0) {
      /*Processing compensation equations*/
      process_comp_y1 = (((int32_t)trim_data.dig_xyz1) * 16384) / process_comp_y0;
      process_comp_y2 = ((uint16_t)process_comp_y1) - ((uint16_t)0x4000);
      retval = ((int16_t)process_comp_y2);
      process_comp_y3 = ((int32_t)retval) * ((int32_t)retval);
      process_comp_y4 = ((int32_t)trim_data.dig_xy2) * (process_comp_y3 / 128);
      process_comp_y5 = ((int32_t)(((int16_t)trim_data.dig_xy1) * 128));
      process_comp_y6 = ((process_comp_y4 + (((int32_t)retval) * process_comp_y5)) / 512);
      process_comp_y7 = ((int32_t)(((int16_t)trim_data.dig_y2) + ((int16_t)0xA0)));
      process_comp_y8 = (((process_comp_y6 + ((int32_t)0x100000)) * process_comp_y7) / 4096);
      process_comp_y9 = (((int32_t)mag_data_y) * process_comp_y8);
      retval = (int16_t)(process_comp_y9 / 8192);
      retval = (retval + (((int16_t)trim_data.dig_y1) * 8)) / 16;
    }
    else {
      retval = OVERFLOW_OUTPUT;
    }
  }
  else {
    /* Overflow condition*/
    retval = OVERFLOW_OUTPUT;
  }

  return retval;
}

int16_t BMM150::compensateZ(const int16_t& mag_data_z, const uint16_t& data_r_hall)
{
  int32_t retval;
  int16_t process_comp_z0;
  int32_t process_comp_z1;
  int32_t process_comp_z2;
  int32_t process_comp_z3;
  int16_t process_comp_z4;

  if (mag_data_z != ZAXIS_HALL_OVERFLOW_ADCVAL) {
    if ((trim_data.dig_z2 != 0) && (trim_data.dig_z1 != 0) && (data_r_hall != 0) && (trim_data.dig_xyz1 != 0)) {
      /*Processing compensation equations*/
      process_comp_z0 = ((int16_t)data_r_hall) - ((int16_t)trim_data.dig_xyz1);
      process_comp_z1 = (((int32_t)trim_data.dig_z3) * ((int32_t)(process_comp_z0))) / 4;
      process_comp_z2 = (((int32_t)(mag_data_z - trim_data.dig_z4)) * 32768);
      process_comp_z3 = ((int32_t)trim_data.dig_z1) * (((int16_t)data_r_hall) * 2);
      process_comp_z4 = (int16_t)((process_comp_z3 + (32768)) / 65536);
      retval = ((process_comp_z2 - process_comp_z1) / (trim_data.dig_z2 + process_comp_z4));

      /* saturate result to +/- 2 micro-tesla */
      if (retval > POSITIVE_SATURATION_Z) {
        retval = POSITIVE_SATURATION_Z;
      }
      else {
        if (retval < NEGATIVE_SATURATION_Z) {
          retval = NEGATIVE_SATURATION_Z;
        }
      }
      /* Conversion of LSB to micro-tesla*/
      retval = retval / 16;
    }
    else {
      retval = OVERFLOW_OUTPUT;
    }
  }
  else {
    /* Overflow condition*/
    retval = OVERFLOW_OUTPUT;
  }

  return (int16_t)retval;
}

}  // namespace driver
