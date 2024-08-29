#include <iostream>

#include "tobas_setup_assistant/robot_info.hpp"

using namespace std;

namespace gui
{
namespace setup_assistant
{
RobotInfo::RobotInfo()
{
}

void RobotInfo::loadFromPath(const string& path)
{
  cout << path << endl;
  // TODO
}

void RobotInfo::loadFromString(const string& xml)
{
  cout << xml << endl;
  // TODO
}

const kdl::Tree& RobotInfo::tree() const
{
  return tree_;
}

const hardware_interface::HardwareInfo& RobotInfo::hardware() const
{
  return hardware_;
}

hw_interface::type_t RobotInfo::hardwareInterface(const string& jnt_name) const
{
  for (const auto& transmission : hardware_.transmissions)
  {
    for (const auto& joint : transmission.joints)
    {
      if (joint.name == jnt_name)
      {
        const auto& hi = transmission.type;
        if (hi == hw_interface::kPositionInterface)
        {
          return hw_interface::POSITION;
        }
        else if (hi == hw_interface::kVelocityInterface)
        {
          return hw_interface::VELOCITY;
        }
        else if (hi == hw_interface::kEffortInterface)
        {
          return hw_interface::EFFORT;
        }
        else
        {
          cerr << "Invalid hardware interface of joint " << jnt_name << ": " << hi << endl;
          return hw_interface::UNKNOWN;
        }
      }
    }
  }

  return hw_interface::NONE;
}
}  // namespace setup_assistant
}  // namespace gui
