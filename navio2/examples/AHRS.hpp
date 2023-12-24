#pragma once

#include <cmath>
#include <memory>
#include <stdio.h>

#include <Common/InertialSensor.h>

class AHRS
{
private:
  float q0, q1, q2, q3;
  float gyroOffset[3];
  float twoKi;
  float twoKp;
  float integralFBx, integralFBy, integralFBz;
  std::unique_ptr<InertialSensor> sensor;

public:
  AHRS(std::unique_ptr<InertialSensor> imu);

  void update(float dt);
  void updateIMU(float dt);
  void setGyroOffset();
  void getEuler(float* roll, float* pitch, float* yaw);

  float invSqrt(float x);
  float getW();
  float getX();
  float getY();
  float getZ();
};
