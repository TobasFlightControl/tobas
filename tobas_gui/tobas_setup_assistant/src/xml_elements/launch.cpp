// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/xml_elements/launch.hpp"

#include <tobas_std_tools/check.hpp>

namespace tobas
{
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
  const std::string& name,
  const std::string& ns,
  const std::string& output,
  const std::string& args)
{
  const auto node = parent->InsertNewChildElement("node");
  node->SetAttribute("pkg", pkg.c_str());
  node->SetAttribute("exec", exec.c_str());
  if (!name.empty()) {
    node->SetAttribute("name", name.c_str());
  }
  if (!ns.empty()) {
    node->SetAttribute("namespace", ns.c_str());
  }
  if (!output.empty()) {
    node->SetAttribute("output", output.c_str());
  }
  if (!args.empty()) {
    node->SetAttribute("args", args.c_str());
  }
  return node;
}

tinyxml2::XMLElement* addNodeParam(tinyxml2::XMLElement* node, const std::string& name, const std::string& value)
{
  TOBAS_CHECK(strcmp(node->Name(), "node") == 0);

  const auto param = node->InsertNewChildElement("param");
  param->SetAttribute("name", name.c_str());
  param->SetAttribute("value", value.c_str());

  return param;
}
}  // namespace xml
}  // namespace sa
}  // namespace gui
}  // namespace tobas
