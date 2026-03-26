#pragma once

#include <map>

#include <urdf/model.h>

#include "./control_surface.hpp"
#include "./thrust.hpp"
#include "./tilt_rotor.hpp"

namespace tobas
{
namespace uadf
{
class Model
{
public:
  urdf::ModelInterfaceSharedPtr urdf;

  std::map<std::string, Thrust> thrusts;
  std::map<std::string, ControlSurface> control_surfaces;
  std::map<std::string, TiltJoint> tilts;

  explicit Model();

  void clear();

  bool valid() const;
};
}  // namespace uadf
}  // namespace tobas
