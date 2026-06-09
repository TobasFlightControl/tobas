// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_uadf/exporter.hpp"

#include <tobas_urdf/exporter.hpp>

namespace tobas
{
namespace uadf
{
tinyxml2::XMLDocument* exportUADF(const Model& model)
{
  // 基となるXMLを作成
  const auto doc = urdf::exportUrdf(*model.urdf);

  // ルートノードを取得
  const auto robot = doc->RootElement();

  // XMLに特殊なジョイント型を埋め込む
  for (auto child = robot->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (std::strcmp(child->Name(), "joint") == 0) {
      const auto joint_name = child->Attribute("name");

      if (model.thrusts.contains(joint_name)) {
        child->SetAttribute("type", "thrust");

        const auto& thrust = model.thrusts.at(joint_name);

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
      else if (model.control_surfaces.contains(joint_name)) {
        child->SetAttribute("type", "cs");
      }
      else if (model.tilts.contains(joint_name)) {
        child->SetAttribute("type", "tilt");
      }
    }
  }

  return doc;
}
}  // namespace uadf
}  // namespace tobas
