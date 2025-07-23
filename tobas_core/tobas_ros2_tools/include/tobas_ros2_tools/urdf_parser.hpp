#pragma once

#include <urdf_world/types.h>

#include <tobas_ros2_tools/console_bridge/output_handler_text.hpp>

namespace ros2
{
class UrdfParser
{
public:
  explicit UrdfParser();

  urdf::ModelInterfaceSharedPtr parseFromPath(const std::string& path);
  urdf::ModelInterfaceSharedPtr parseFromText(const std::string& xml);

  const std::string& errorMessage() const;

private:
  std::string error_msg_;

  console_bridge::OutputHandlerText oh_;
};
}  // namespace ros2
