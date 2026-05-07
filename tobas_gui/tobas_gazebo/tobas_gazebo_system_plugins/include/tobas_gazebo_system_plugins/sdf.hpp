// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/common/Console.hh>
#include <gz/math/Vector4.hh>
#include <sdf/sdf.hh>

namespace tobas
{
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

template <typename T>
bool getSdfParam(const sdf::ElementConstPtr& sdf, const std::string& name, gz::math::Vector4<T>& param)
{
  // 文字列として取得 (e.g. "1.0 2.0 3.0 0.5")
  std::string str;
  if (!getSdfParam(sdf, name, str)) {
    return false;
  }

  std::istringstream iss(str);
  T x, y, z, w;
  if (!(iss >> x >> y >> z >> w)) {
    gzerr << "Invalid vec4: " << str << std::endl;
    return false;
  }

  param.X(x);
  param.Y(y);
  param.Z(z);
  param.W(w);

  return true;
}

bool getTurningDirection(const sdf::ElementConstPtr& sdf, int& dst);
}  // namespace gazebo
}  // namespace tobas
