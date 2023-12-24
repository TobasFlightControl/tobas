#pragma once

class RCInput
{
public:
  virtual void initialize() = 0;
  virtual int read(int c) = 0;
};
