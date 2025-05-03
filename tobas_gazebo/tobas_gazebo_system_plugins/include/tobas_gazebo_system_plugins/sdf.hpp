#pragma once

#include <sdf/sdf.hh>
#include <gz/common/Console.hh>

namespace gazebo
{
template <typename T>
bool getSdfParam(const sdf::ElementConstPtr& sdf, const std::string& name, T& param)
{
  if (!sdf->HasElement(name)) {
    gzerr << "Please specify \"" << name << "\"." << std::endl;
    return false;
  }

  param = sdf->Get<T>(name);
  return true;
}

template <typename T>
void getSdfParam(const sdf::ElementConstPtr& sdf, const std::string& name, T& param, const T& dflt)
{
  if (!sdf->Get(name, param, dflt)) {
    gzwarn << "SDF parameter \"" << name << "\" is not specified. The default value \"" << dflt << "\" is used."
           << std::endl;
  }
}

template <typename T>
bool getSdfParam(const sdf::ElementConstPtr& sdf, const std::string& name, std::pair<T, T>& param)
{
  gz::math::Vector2<T> tmp;
  if (!getSdfParam(sdf, name, tmp)) {
    return false;
  }

  param.first = tmp.X();
  param.second = tmp.Y();

  return true;
}

bool getTurningDirection(const sdf::ElementConstPtr& sdf, int& dst);
}  // namespace gazebo
