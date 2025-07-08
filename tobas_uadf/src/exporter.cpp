#include "tobas_uadf/exporter.hpp"

#include <urdf_parser/urdf_parser.h>

namespace uadf
{
tinyxml2::XMLDocument* exportUADF(const Model& model)
{
  // 基となるXMLを作成
  const auto doc = urdf::exportURDF(*model.urdf);
  const auto robot = doc->RootElement();

  // XMLに特殊なジョイント型を埋め込む
  for (auto child = robot->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (strcmp(child->Name(), "joint") == 0) {
      const auto joint_name = child->Attribute("name");

      if (model.thrusts.contains(joint_name)) {
        child->SetAttribute("type", "thrust");

        const auto& thrust = model.thrusts.at(joint_name);

        const auto direction = child->InsertNewChildElement("direction");
        switch (thrust.direction) {
          case Thrust::CW:
            direction->SetAttribute("value", "cw");
          case Thrust::CCW:
            direction->SetAttribute("value", "ccw");
          default:
            throw;
        }
      }
      else if (model.tilts.contains(joint_name)) {
        child->SetAttribute("type", "tilt");
      }
      else if (model.control_surfaces.contains(joint_name)) {
        child->SetAttribute("type", "cs");
      }
    }
  }

  return doc;
}
}  // namespace uadf
