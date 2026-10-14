// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_uadf/exporter.hpp"

#include <tobas_urdf/exporter.hpp>

namespace tobas::uadf
{
tinyxml2::XMLDocument* exportUADF(const Model& _model)
{
  // Create the base XML.
  const auto doc = urdf::exportUrdf(*_model.urdf);

  // Get the root node.
  const auto robot = doc->RootElement();

  // Embed the special joint types in the XML.
  for (auto child = robot->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (std::strcmp(child->Name(), "joint") == 0) {
      const auto joint_name = child->Attribute("name");

      if (_model.thrusts.contains(joint_name)) {
        child->SetAttribute("type", "thrust");

        const auto& thrust = _model.thrusts.at(joint_name);

        const auto direction = child->InsertNewChildElement("direction");
        switch (thrust.direction) {
          case Thrust::CW:
            direction->SetAttribute("value", "cw");
            break;
          case Thrust::CCW:
            direction->SetAttribute("value", "ccw");
            break;
          default:
            throw;
        }
      }
      else if (_model.control_surfaces.contains(joint_name)) {
        child->SetAttribute("type", "cs");
      }
      else if (_model.tilts.contains(joint_name)) {
        child->SetAttribute("type", "tilt");
      }
    }
  }

  return doc;
}
}  // namespace tobas::uadf
