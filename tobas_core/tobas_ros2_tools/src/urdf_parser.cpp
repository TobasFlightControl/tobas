#include "tobas_ros2_tools/urdf_parser.hpp"

#include <urdf_parser/urdf_parser.h>

namespace ros2
{
UrdfParser::UrdfParser() : oh_(console_bridge::CONSOLE_BRIDGE_LOG_ERROR)
{
}

urdf::ModelInterfaceSharedPtr UrdfParser::parseFromPath(const std::string& path)
{
  console_bridge::useOutputHandler(&oh_);

  const auto res = urdf::parseURDFFile(path);
  if (!res) {
    error_msg_ = oh_.message();
    oh_.clear();
  }

  console_bridge::restorePreviousOutputHandler();

  return res;
}

urdf::ModelInterfaceSharedPtr UrdfParser::parseFromText(const std::string& xml)
{
  console_bridge::useOutputHandler(&oh_);

  const auto res = urdf::parseURDF(xml);
  if (!res) {
    error_msg_ = oh_.message();
    oh_.clear();
  }

  console_bridge::restorePreviousOutputHandler();

  return res;
}

const std::string& UrdfParser::errorMessage() const
{
  return error_msg_;
}
}  // namespace ros2
