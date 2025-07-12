#include "tobas_uadf/exporter.hpp"

#include <urdf_parser/urdf_parser.h>

namespace uadf
{
tinyxml2::XMLDocument* exportUADF(const Model& model)
{
// 基となるXMLを作成
// FIXME: Avoid deprecated function
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  const auto doc = urdf::exportURDF(*model.urdf);
#pragma GCC diagnostic pop

  // ルートノードを取得
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
