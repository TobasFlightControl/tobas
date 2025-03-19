#pragma once

namespace dsp
{
template <typename T>
class BaseFilter
{
public:
  virtual void update(const T& u, const double& dt) = 0;

  virtual const T& getValue() const = 0;
  virtual void setValue(const T& x) = 0;
};
}  // namespace dsp
