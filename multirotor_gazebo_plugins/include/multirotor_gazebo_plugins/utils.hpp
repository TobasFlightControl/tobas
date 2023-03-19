#pragma once

namespace gazebo
{
template <typename T>
T sqr(const T& x)
{
  return x * x;
}

template <class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string& name, T& param, const T& default_value)
{
  if (sdf->HasElement(name))
  {
    param = sdf->GetElement(name)->Get<T>();
    return true;
  }
  else
  {
    param = default_value;
    return false;
  }
}
}  // namespace gazebo
