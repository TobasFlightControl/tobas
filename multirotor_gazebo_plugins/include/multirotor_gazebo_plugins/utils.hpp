#pragma once

#include <string>
#include <boost/array.hpp>
#include <gazebo/gazebo.hh>

/**
 * @brief utilities
 * @note 各プラグインは個別にビルドしてリンクは行っていないため，ヘッダオンリーで書く．
 */
namespace gazebo
{
template <typename T>
inline T sqr(const T& x)
{
  return x * x;
}

template <typename T>
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

template <typename T>
bool allGreaterEqual(const ignition::math::Vector3<T>& v, T x)
{
  return v.X() >= x && v.Y() >= x && v.Z() >= x;
}

/* 3x3行列の対角要素を埋める． */
template <typename T>
void fillMatrix3Diag(boost::array<T, 9>& m, T v)
{
  m[0] = v;
  m[4] = v;
  m[8] = v;
}
}  // namespace gazebo
