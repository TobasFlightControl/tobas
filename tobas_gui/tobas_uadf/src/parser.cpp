// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_uadf/parser.hpp"

#include <tobas_string_tools/stream.hpp>
#include <tobas_xml_tools/core.hpp>

namespace tobas
{
namespace uadf
{
Parser::Parser()
{
}

std::expected<Model, std::string> Parser::parseFromXml(const tinyxml2::XMLDocument* uadf_doc)
{
  // Initialize the model.
  Model uadf_model;

  // Copy the XML document for editing.
  tinyxml2::XMLDocument uadf_doc_cp;
  uadf_doc->DeepCopy(&uadf_doc_cp);

  // Get the root element.
  const auto robot = uadf_doc_cp.RootElement();

  // Convert special joint types to URDF.
  for (auto child = robot->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (std::strcmp(child->Name(), "joint") == 0) {
      const auto joint_name = child->Attribute("name");
      const auto joint_type = child->Attribute("type");

      if (std::strcmp(joint_type, "thrust") == 0) {
        child->SetAttribute("type", "continuous");

        Thrust thrust;
        bool direction_found = false;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {
          if (std::strcmp(gchild->Name(), "direction") == 0) {
            direction_found = true;
            const auto direction = gchild->Attribute("value");
            if (std::strcmp(direction, "cw") == 0) {
              thrust.direction = Thrust::CW;
            }
            else if (std::strcmp(direction, "ccw") == 0) {
              thrust.direction = Thrust::CCW;
            }
            else {
              return std::unexpected(
                "Thrust joint '" + std::string(joint_name) + "' has invalid direction '" + std::string(direction) +
                "'. It must be 'cw' or 'ccw'.");
            }
          }
        }

        if (!direction_found) {
          return std::unexpected("Thrust joint '" + std::string(joint_name) + "' has no 'direction' element.");
        }

        uadf_model.thrusts[joint_name] = thrust;
      }
      else if (std::strcmp(joint_type, "cs") == 0) {
        child->SetAttribute("type", "revolute");

        ControlSurface cs;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {}

        uadf_model.control_surfaces[joint_name] = cs;
      }
      else if (std::strcmp(joint_type, "tilt") == 0) {
        child->SetAttribute("type", "revolute");

        TiltJoint tilt;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {}

        uadf_model.tilts[joint_name] = tilt;
      }
    }
  }

  // Write the URDF.
  const auto urdf_text = xml::xmlDocumentToString(&uadf_doc_cp);

  // Parse the URDF.
  const auto urdf_model = urdf_parser_.parseFromText(urdf_text);
  if (!urdf_model) {
    return std::unexpected(urdf_model.error());
  }
  uadf_model.urdf = *urdf_model;

  return uadf_model;
}

std::expected<Model, std::string> Parser::parseFromText(const std::string& uadf_text)
{
  tinyxml2::XMLDocument uadf_doc;
  if (uadf_doc.Parse(uadf_text.c_str()) != tinyxml2::XML_SUCCESS) {
    return std::unexpected(uadf_doc.ErrorStr());
  }

  return parseFromXml(&uadf_doc);
}

std::expected<Model, std::string> Parser::parseFromPath(const std::string& uadf_path)
{
  std::string uadf_text;
  if (!str::readText(uadf_path, uadf_text)) {
    return std::unexpected("Failed to open file: " + uadf_path);
  }

  return parseFromText(uadf_text);
}
}  // namespace uadf
}  // namespace tobas
