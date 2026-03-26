#include "tobas_gazebo_gui_plugins/utils.hpp"

#include <gz/gui/Helpers.hh>

namespace tobas
{
namespace gazebo
{
std::expected<std::string, std::string> getWorldName()
{
  const auto world_names = gz::gui::worldNames();
  if (world_names.empty()) {
    return std::unexpected("Failed to get world names.");
  }
  return world_names[0].toStdString();
}
}  // namespace gazebo
}  // namespace tobas
