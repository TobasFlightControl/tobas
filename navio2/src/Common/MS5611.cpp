#include <math.h>
#include <unistd.h>

#include "../../include/Common/MS5611.h"

MS5611::MS5611(uint8_t address)
{
  devAddr = address;
}

void MS5611::initialize()
{
  // Reading 6 calibration data values
  uint8_t buff[2];
  I2Cdev::readBytes(devAddr, MS5611_RA_C1, 2, buff);
  C1 = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(devAddr, MS5611_RA_C2, 2, buff);
  C2 = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(devAddr, MS5611_RA_C3, 2, buff);
  C3 = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(devAddr, MS5611_RA_C4, 2, buff);
  C4 = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(devAddr, MS5611_RA_C5, 2, buff);
  C5 = buff[0] << 8 | buff[1];
  I2Cdev::readBytes(devAddr, MS5611_RA_C6, 2, buff);
  C6 = buff[0] << 8 | buff[1];

  update();
}

bool MS5611::testConnection()
{
  uint8_t data;
  int8_t status = I2Cdev::readByte(devAddr, MS5611_RA_C0, &data);
  return status > 0;
}

void MS5611::refreshPressure(uint8_t OSR)
{
  I2Cdev::writeBytes(devAddr, OSR, 0, 0);
}

void MS5611::readPressure()
{
  uint8_t buffer[3];
  I2Cdev::readBytes(devAddr, MS5611_RA_ADC, 3, buffer);
  D1 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
}

void MS5611::refreshTemperature(uint8_t OSR)
{
  I2Cdev::writeBytes(devAddr, OSR, 0, 0);
}

void MS5611::readTemperature()
{
  uint8_t buffer[3];
  I2Cdev::readBytes(devAddr, MS5611_RA_ADC, 3, buffer);
  D2 = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
}

void MS5611::calculatePressureAndTemperature()
{
  float dT = D2 - C5 * pow(2, 8);
  TEMP = (2000 + ((dT * C6) / pow(2, 23)));
  float OFF = C2 * pow(2, 16) + (C4 * dT) / pow(2, 7);
  float SENS = C1 * pow(2, 15) + (C3 * dT) / pow(2, 8);

  float T2, OFF2, SENS2;

  if (TEMP >= 2000)
  {
    T2 = 0;
    OFF2 = 0;
    SENS2 = 0;
  }
  if (TEMP < 2000)
  {
    T2 = dT * dT / pow(2, 31);
    OFF2 = 5 * pow(TEMP - 2000, 2) / 2;
    SENS2 = OFF2 / 2;
  }
  if (TEMP < -1500)
  {
    OFF2 = OFF2 + 7 * pow(TEMP + 1500, 2);
    SENS2 = SENS2 + 11 * pow(TEMP + 1500, 2) / 2;
  }

  TEMP = TEMP - T2;
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;

  // Final calculations
  PRES = ((D1 * SENS) / pow(2, 21) - OFF) / pow(2, 15) / 100;
  TEMP = TEMP / 100;
}

void MS5611::update()
{
  refreshPressure();
  usleep(10000);  // Waiting for pressure data ready
  readPressure();

  refreshTemperature();
  usleep(10000);  // Waiting for temperature data ready
  readTemperature();

  calculatePressureAndTemperature();
}

float MS5611::getTemperature()
{
  return TEMP;
}

float MS5611::getPressure()
{
  return PRES;
}
