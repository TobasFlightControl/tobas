#include "../../include/tobas_drone_core/joint/role.hpp"

namespace tobas
{
bool isServoJoint(jnt_role_t role)
{
  switch (role)
  {
    case jnt_role_t::ROTOR:
      return false;
    case jnt_role_t::TILT_JOINT:
      return true;
    case jnt_role_t::CONTROL_SURFACE:
      return true;
    case jnt_role_t::MANIPULATION:
      return true;
    case jnt_role_t::WHEEL:
      return false;
    case jnt_role_t::OTHER:
      return false;
    default:
      throw;
  }
}
}  // namespace tobas
