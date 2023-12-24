#pragma once

class InertialSensor
{
public:
  virtual void initialize() = 0;
  virtual bool probe() = 0;
  virtual void update() = 0;

  float readTemperature()
  {
    return temperature;
  };

  void readAccelerometer(float* ax, float* ay, float* az)
  {
    *ax = ax_;
    *ay = ay_;
    *az = az_;
  };

  void readGyroscope(float* gx, float* gy, float* gz)
  {
    *gx = gx_;
    *gy = gy_;
    *gz = gz_;
  };

  void readMagnetometer(float* mx, float* my, float* mz)
  {
    *mx = mx_;
    *my = my_;
    *mz = mz_;
  };

protected:
  float temperature;
  float ax_, ay_, az_;
  float gx_, gy_, gz_;
  float mx_, my_, mz_;
};
