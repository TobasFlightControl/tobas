#pragma once

#include <tinyxml2.h>

#include <tobas_ros2_tools/console_bridge/output_handler_text.hpp>

#include "./model.hpp"

namespace uadf
{
class Parser
{
public:
  explicit Parser();

  bool parseFromXml(const tinyxml2::XMLDocument* uadf_doc, Model& uadf_model);
  bool parseFromText(const std::string& uadf_text, Model& uadf_model);
  bool parseFromPath(const std::string& uadf_path, Model& uadf_model);

  const std::string& errorMessage() const;

private:
  std::string error_msg_;

  console_bridge::OutputHandlerText oh_;
};
}  // namespace uadf
