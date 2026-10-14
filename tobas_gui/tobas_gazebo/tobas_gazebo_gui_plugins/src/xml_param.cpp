// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_gui_plugins/xml_param.hpp"

#include <gz/common/Console.hh>

namespace tobas::gazebo
{
bool getXmlParam(const tinyxml2::XMLElement* _elem, const char* _name, double& _param)
{
  const auto child_elem = _elem->FirstChildElement(_name);
  if (!child_elem) {
    gzerr << "XML parameter '" << _name << "' does not exist.\n";
    return false;
  }

  const std::string text(child_elem->GetText());

  try {
    _param = stod(text);
  }
  catch (const std::exception& e) {
    gzerr << e.what() << '\n';
    return false;
  }

  return true;
}
}  // namespace tobas::gazebo
