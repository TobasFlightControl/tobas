#pragma once

#include <gazebo/gazebo.hh>

/**
 * @brief utilities
 * @note 各プラグインは個別にビルドしてリンクは行っていないため，ヘッダオンリーで書く．
 */
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

bool allGreaterEqual(const ignition::math::Vector3d& v, double x)
{
  return v.X() >= x && v.Y() >= x && v.Z() >= x;
}
}  // namespace gazebo
