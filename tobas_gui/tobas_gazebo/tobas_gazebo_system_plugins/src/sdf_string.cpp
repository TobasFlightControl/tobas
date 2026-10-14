// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/sdf_string.hpp"

#include <format>

#include "tobas_gazebo_system_plugins/inertia.hpp"

namespace tobas::gazebo
{
std::string makeBoxSdf(
  const std::string& _name,
  double _sx,
  double _sy,
  double _sz,
  double _mass,
  double _px,
  double _py,
  double _pz,
  double _rr,
  double _rp,
  double _ry)
{
  const auto inertia = boxInertia(_sx, _sy, _sz, _mass);

  return std::format(
    R"(<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{0}">
    <static>false</static>
    <link name="link">
      <pose>{1} {2} {3} {4} {5} {6}</pose>

      <inertial>
        <mass>{7}</mass>
        <inertia>
          <ixx>{8}</ixx>
          <iyy>{9}</iyy>
          <izz>{10}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box><size>{11} {12} {13}</size></box>
        </geometry>
      </collision>

      <visual name="visual">
        <geometry>
          <box><size>{11} {12} {13}</size></box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
)",
    _name,
    _px,
    _py,
    _pz,
    _rr,
    _rp,
    _ry,
    _mass,
    inertia.Ixx(),
    inertia.Iyy(),
    inertia.Izz(),
    _sx,
    _sy,
    _sz);
}
}  // namespace tobas::gazebo
