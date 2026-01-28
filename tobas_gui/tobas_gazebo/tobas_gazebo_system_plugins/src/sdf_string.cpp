#include "tobas_gazebo_system_plugins/sdf_string.hpp"

#include <format>

#include "tobas_gazebo_system_plugins/inertia.hpp"

namespace gazebo
{
std::string makeBoxSdf(
  const std::string& name,
  double sx,
  double sy,
  double sz,
  double mass,
  double px,
  double py,
  double pz,
  double rr,
  double rp,
  double ry)
{
  const auto [ixx, iyy, izz] = boxInertia(sx, sy, sz, mass);

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
    name,
    px,
    py,
    pz,
    rr,
    rp,
    ry,
    mass,
    ixx,
    iyy,
    izz,
    sx,
    sy,
    sz);
}
}  // namespace gazebo
