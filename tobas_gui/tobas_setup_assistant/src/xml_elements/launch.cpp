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
  tinyxml2::XMLElement* _parent,
  const std::string& _pkg,
  const std::string& _exec,
  const std::string& _name,
  const std::string& _ns,
  const std::string& _output,
  const std::string& _args)
{
  const auto node = _parent->InsertNewChildElement("node");
  node->SetAttribute("pkg", _pkg.c_str());
  node->SetAttribute("exec", _exec.c_str());
  if (!_name.empty()) {
    node->SetAttribute("name", _name.c_str());
  }
  if (!_ns.empty()) {
    node->SetAttribute("namespace", _ns.c_str());
  }
  if (!_output.empty()) {
    node->SetAttribute("output", _output.c_str());
  }
  if (!_args.empty()) {
    node->SetAttribute("args", _args.c_str());
  }
  return node;
}

tinyxml2::XMLElement* addNodeParam(tinyxml2::XMLElement* _node, const std::string& _name, const std::string& _value)
{
  TOBAS_CHECK(std::strcmp(_node->Name(), "node") == 0);

  const auto param = _node->InsertNewChildElement("param");
  param->SetAttribute("name", _name.c_str());
  param->SetAttribute("value", _value.c_str());

  return param;
}
}  // namespace xml
}  // namespace sa
}  // namespace gui
}  // namespace tobas
