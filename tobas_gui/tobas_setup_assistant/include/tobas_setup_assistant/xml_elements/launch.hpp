#pragma once

#include <string>

#include <tinyxml2.h>

namespace gui
{
namespace sa
{
namespace xml
{
tinyxml2::XMLElement* addNode(
  tinyxml2::XMLElement* parent,
  const std::string& pkg,
  const std::string& exec,
  const std::string& name = "",
  const std::string& ns = "",
  const std::string& output = "screen",
  const std::string& args = "");

tinyxml2::XMLElement* addNodeParam(tinyxml2::XMLElement* node, const std::string& name, const std::string& value);
}  // namespace xml
}  // namespace sa
}  // namespace gui
