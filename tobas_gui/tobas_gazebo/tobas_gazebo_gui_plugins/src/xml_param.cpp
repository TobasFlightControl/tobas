// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_gui_plugins/xml_param.hpp"

#include <gz/common/Console.hh>

namespace tobas
{
namespace gazebo
{
bool getXmlParam(const tinyxml2::XMLElement* elem, const char* name, double& param)
{
  const auto child_elem = elem->FirstChildElement(name);
  if (!child_elem) {
    gzerr << "XML parameter \"" << name << "\" does not exist." << std::endl;
    return false;
  }

  const std::string text(child_elem->GetText());

  try {
    param = stod(text);
  }
  catch (const std::exception& e) {
    gzerr << e.what() << std::endl;
    return false;
  }

  return true;
}
}  // namespace gazebo
}  // namespace tobas
