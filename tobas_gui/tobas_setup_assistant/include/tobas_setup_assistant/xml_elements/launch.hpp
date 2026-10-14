// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

#include <tinyxml2.h>

namespace tobas::gui::sa::xml
{
tinyxml2::XMLElement* addNode(
  tinyxml2::XMLElement* _parent,
  const std::string& _pkg,
  const std::string& _exec,
  const std::string& _name = "",
  const std::string& _ns = "",
  const std::string& _output = "screen",
  const std::string& _args = "");

tinyxml2::XMLElement* addNodeParam(tinyxml2::XMLElement* _node, const std::string& _name, const std::string& _value);
}  // namespace tobas::gui::sa::xml
