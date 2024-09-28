#include "tobas_control_system/rotors_viewer/rotors_viewer.hpp"

namespace gui
{
namespace control_system
{
RotorsViewerWiddget::RotorsViewerWiddget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  // TODO
}

void RotorsViewerWiddget::updateInternalDataStructures()
{
  // TODO
}
}  // namespace control_system
}  // namespace gui
