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
bool getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, T& _param)
{
  if (!_sdf->HasElement(_name)) {
    gzerr << "Please specify '" << _name << "'.\n";
    return false;
  }

  _param = _sdf->Get<T>(_name);
  return true;
}

template <typename T>
void getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, T& _param, const T& _dflt)
{
  if (!_sdf->Get(_name, _param, _dflt)) {
    gzwarn << "SDF parameter '" << _name << "' is not specified. The default value '" << _dflt << "' is used.\n";
  }
}

template <typename T>
bool getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, std::pair<T, T>& _param)
{
  gz::math::Vector2<T> tmp;
  if (!getSdfParam(_sdf, _name, tmp)) {
    return false;
  }

  _param.first = tmp.X();
  _param.second = tmp.Y();

  return true;
}

template <typename T>
bool getSdfParam(const sdf::ElementConstPtr& _sdf, const std::string& _name, gz::math::Vector4<T>& _param)
{
  // Get as a string (e.g. "1.0 2.0 3.0 0.5").
  std::string str;
  if (!getSdfParam(_sdf, _name, str)) {
    return false;
  }

  std::istringstream iss(str);
  T x, y, z, w;
  if (!(iss >> x >> y >> z >> w)) {
    gzerr << "Invalid vec4: " << str << '\n';
    return false;
  }

  _param.X(x);
  _param.Y(y);
  _param.Z(z);
  _param.W(w);

  return true;
}

bool getTurningDirection(const sdf::ElementConstPtr& _sdf, int& _dst);
}  // namespace gazebo
}  // namespace tobas
