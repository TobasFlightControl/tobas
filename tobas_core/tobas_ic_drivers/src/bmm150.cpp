#include <iostream>
#include <bitset>
#include <thread>
#include <cstring>

#include "../include/tobas_ic_drivers/bmm150.hpp"

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

  int16_t rawDataX = ((i2c_.rx[1] << 5) | (i2c_.rx[0] >> 3));
  int16_t rawDataY = ((i2c_.rx[3] << 5) | (i2c_.rx[2] >> 3));
  int16_t rawDataZ = ((i2c_.rx[5] << 7) | (i2c_.rx[4] >> 1));
  uint16_t rHall = ((i2c_.rx[7] << 6) | (i2c_.rx[6] >> 2));

  // TODO: compensate for the temperature effect using resistance value of hall sensor based on
  // https://github.com/boschsensortec/BMM150_SensorAPI/blob/master/bmm150.c compensate_x
  mx = static_cast<double>(compensateX(rawDataX, rHall)) / 16.0;
  my = static_cast<double>(compensateY(rawDataY, rHall)) / 16.0;
  mz = static_cast<double>(compensateZ(rawDataZ, rHall)) / 16.0;

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

// ref: https://github.com/boschsensortec/BMM150_SensorAPI/blob/master/bmm150.c (not written in data sheet)
bool BMM150::readTrimRegisters()
{
  uint8_t trimX1y1[2] = { 0 };
  uint8_t trimXyzData[4] = { 0 };
  uint8_t trimXy1xy2[10] = { 0 };
  uint16_t tempMsb = 0;

  /* Trim register value is read */
  if (!readRegs(DIG_X1_REG, 2)) {
    cerr << "Failed to read DIG_X1_REG." << endl;
    return false;
  }
  memcpy(trimX1y1, i2c_.rx, 2);
  if (!readRegs(DIG_Z4_LSB_REG, 4)) {
    cerr << "Failed to read DIG_Z4_REG." << endl;
    return false;
  }
  memcpy(trimXyzData, i2c_.rx, 4);
  if (!readRegs(DIG_Z2_LSB_REG, 10)) {
    cerr << "Failed to read DIG_Z2_LSB_REG." << endl;
    return false;
  }
  memcpy(trimXy1xy2, i2c_.rx, 10);

  /*  Trim data which is read is updated
      in the device structure */
  trimData.digX1 = (int8_t)trimX1y1[0];
  trimData.digY1 = (int8_t)trimX1y1[1];
  trimData.digX2 = (int8_t)trimXyzData[2];
  trimData.digY2 = (int8_t)trimXyzData[3];
  tempMsb = ((uint16_t)trimXy1xy2[3]) << 8;
  trimData.digZ1 = (uint16_t)(tempMsb | trimXy1xy2[2]);
  tempMsb = ((uint16_t)trimXy1xy2[1]) << 8;
  trimData.digZ2 = (int16_t)(tempMsb | trimXy1xy2[0]);
  tempMsb = ((uint16_t)trimXy1xy2[7]) << 8;
  trimData.digZ3 = (int16_t)(tempMsb | trimXy1xy2[6]);
  tempMsb = ((uint16_t)trimXyzData[1]) << 8;
  trimData.digZ4 = (int16_t)(tempMsb | trimXyzData[0]);
  trimData.digXy1 = trimXy1xy2[9];
  trimData.digXy2 = (int8_t)trimXy1xy2[8];
  tempMsb = ((uint16_t)(trimXy1xy2[5] & 0x7F)) << 8;
  trimData.digXyz1 = (uint16_t)(tempMsb | trimXy1xy2[4]);

  return true;
}

int16_t BMM150::compensateX(const int16_t& magDataX, const uint16_t& dataRhall)
{
  int16_t retval;
  uint16_t processCompX0 = 0;
  int32_t processCompX1;
  uint16_t processCompX2;
  int32_t processCompX3;
  int32_t processCompX4;
  int32_t processCompX5;
  int32_t processCompX6;
  int32_t processCompX7;
  int32_t processCompX8;
  int32_t processCompX9;
  int32_t processCompX10;

  /* Overflow condition check */
  if (magDataX != XYAXES_FLIP_OVERFLOW_ADCVAL) {
    if (dataRhall != 0) {
      /* Availability of valid data*/
      processCompX0 = dataRhall;
    }
    else if (trimData.digXyz1 != 0) {
      processCompX0 = trimData.digXyz1;
    }
    else {
      processCompX0 = 0;
    }
    if (processCompX0 != 0) {
      /* Processing compensation equations*/
      processCompX1 = ((int32_t)trimData.digXyz1) * 16384;
      processCompX2 = ((uint16_t)(processCompX1 / processCompX0)) - ((uint16_t)0x4000);
      retval = ((int16_t)processCompX2);
      processCompX3 = (((int32_t)retval) * ((int32_t)retval));
      processCompX4 = (((int32_t)trimData.digXy2) * (processCompX3 / 128));
      processCompX5 = (int32_t)(((int16_t)trimData.digXy1) * 128);
      processCompX6 = ((int32_t)retval) * processCompX5;
      processCompX7 = (((processCompX4 + processCompX6) / 512) + ((int32_t)0x100000));
      processCompX8 = ((int32_t)(((int16_t)trimData.digX2) + ((int16_t)0xA0)));
      processCompX9 = ((processCompX7 * processCompX8) / 4096);
      processCompX10 = ((int32_t)magDataX) * processCompX9;
      retval = ((int16_t)(processCompX10 / 8192));
      retval = (retval + (((int16_t)trimData.digX1) * 8)) / 16;
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

/*
  @brief This internal API is used to obtain the compensated
  magnetometer Y axis data(micro-tesla) in int16_t.
*/
int16_t BMM150::compensateY(const int16_t& magDataY, const uint16_t& dataRhall)
{
  int16_t retval;
  uint16_t processCompY0 = 0;
  int32_t processCompY1;
  uint16_t processCompY2;
  int32_t processCompY3;
  int32_t processCompY4;
  int32_t processCompY5;
  int32_t processCompY6;
  int32_t processCompY7;
  int32_t processCompY8;
  int32_t processCompY9;

  /* Overflow condition check */
  if (magDataY != XYAXES_FLIP_OVERFLOW_ADCVAL) {
    if (dataRhall != 0) {
      /* Availability of valid data*/
      processCompY0 = dataRhall;
    }
    else if (trimData.digXyz1 != 0) {
      processCompY0 = trimData.digXyz1;
    }
    else {
      processCompY0 = 0;
    }
    if (processCompY0 != 0) {
      /*Processing compensation equations*/
      processCompY1 = (((int32_t)trimData.digXyz1) * 16384) / processCompY0;
      processCompY2 = ((uint16_t)processCompY1) - ((uint16_t)0x4000);
      retval = ((int16_t)processCompY2);
      processCompY3 = ((int32_t)retval) * ((int32_t)retval);
      processCompY4 = ((int32_t)trimData.digXy2) * (processCompY3 / 128);
      processCompY5 = ((int32_t)(((int16_t)trimData.digXy1) * 128));
      processCompY6 = ((processCompY4 + (((int32_t)retval) * processCompY5)) / 512);
      processCompY7 = ((int32_t)(((int16_t)trimData.digY2) + ((int16_t)0xA0)));
      processCompY8 = (((processCompY6 + ((int32_t)0x100000)) * processCompY7) / 4096);
      processCompY9 = (((int32_t)magDataY) * processCompY8);
      retval = (int16_t)(processCompY9 / 8192);
      retval = (retval + (((int16_t)trimData.digY1) * 8)) / 16;
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

/*
  @brief This internal API is used to obtain the compensated
  magnetometer Z axis data(micro-tesla) in int16_t.
*/
int16_t BMM150::compensateZ(const int16_t& magDataZ, const uint16_t& dataRhall)
{
  int32_t retval;
  int16_t processCompZ0;
  int32_t processCompZ1;
  int32_t processCompZ2;
  int32_t processCompZ3;
  int16_t processCompZ4;

  if (magDataZ != ZAXIS_HALL_OVERFLOW_ADCVAL) {
    if ((trimData.digZ2 != 0) && (trimData.digZ1 != 0) && (dataRhall != 0) && (trimData.digXyz1 != 0)) {
      /*Processing compensation equations*/
      processCompZ0 = ((int16_t)dataRhall) - ((int16_t)trimData.digXyz1);
      processCompZ1 = (((int32_t)trimData.digZ3) * ((int32_t)(processCompZ0))) / 4;
      processCompZ2 = (((int32_t)(magDataZ - trimData.digZ4)) * 32768);
      processCompZ3 = ((int32_t)trimData.digZ1) * (((int16_t)dataRhall) * 2);
      processCompZ4 = (int16_t)((processCompZ3 + (32768)) / 65536);
      retval = ((processCompZ2 - processCompZ1) / (trimData.digZ2 + processCompZ4));

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
