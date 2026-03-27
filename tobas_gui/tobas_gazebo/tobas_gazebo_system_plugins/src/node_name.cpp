#include "tobas_gazebo_system_plugins/node_name.hpp"

#include <cctype>

namespace tobas
{
namespace gazebo
{
std::string sanitizeNodeName(std::string str)
{
  // [0-9A-Za-z_] のみ許可するフィルタ
  const auto is_allowed = [](unsigned char c) { return std::isalnum(c) || c == '_'; };

  for (char& ch : str) {
    if (!is_allowed(static_cast<unsigned char>(ch))) {
      ch = '_';
    }
  }

  return str;
}
}  // namespace gazebo
}  // namespace tobas
