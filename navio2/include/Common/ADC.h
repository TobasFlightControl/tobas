#pragma once

class ADC
{
public:
  virtual void initialize() = 0;
  virtual int get_channel_count(void) = 0;
  virtual int read(int ch) = 0;
};
