#include "tobas_uadf/parser.hpp"

#include <iostream>

#include <urdf_parser/urdf_parser.h>

#include <tobas_ros2_tools/xacro.hpp>
#include <tobas_string_tools/stream.hpp>
#include <tobas_xml_tools/core.hpp>

namespace uadf
{
bool parseFromXml(const tinyxml2::XMLDocument* uadf_doc, Model& uadf_model)
{
  // モデルを初期化
  uadf_model.clear();

  // 編集用にXMLドキュメントをコピー
  tinyxml2::XMLDocument uadf_doc_cp;
  uadf_doc->DeepCopy(&uadf_doc_cp);

  // ルート要素を取得
  const auto robot = uadf_doc_cp.RootElement();
  if (strcmp(robot->Name(), "robot") != 0) {
    std::cerr << "The root name of UADF must be \"robot\"." << std::endl;
    return false;
  }

  // 特殊なジョイント型をURDFに変換
  for (auto child = robot->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (strcmp(child->Name(), "joint") == 0) {
      const auto joint_name = child->Attribute("name");
      const auto joint_type = child->Attribute("type");

      if (strcmp(joint_type, "thrust") == 0) {
        child->SetAttribute("type", "continuous");

        Thrust thrust;
        bool direction_found = false;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {
          if (strcmp(gchild->Name(), "direction") == 0) {
            direction_found = true;
            const auto direction = gchild->Attribute("value");
            if (strcmp(direction, "cw") == 0) {
              thrust.direction = Thrust::CW;
            }
            else if (strcmp(direction, "ccw") == 0) {
              thrust.direction = Thrust::CCW;
            }
            else {
              std::cerr << "Direction must be \"cw\" or \"ccw\"." << std::endl;
              return false;
            }
          }
        }

        if (!direction_found) {
          std::cerr << "Thrust joint \"" << joint_name << "\" does not have any \"direction\" element." << std::endl;
          return false;
        }

        uadf_model.thrusts[joint_name] = thrust;
      }
      else if (strcmp(joint_type, "tilt") == 0) {
        child->SetAttribute("type", "revolute");

        TiltJoint tilt;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {}

        uadf_model.tilts[joint_name] = tilt;
      }
      else if (strcmp(joint_type, "cs") == 0) {
        child->SetAttribute("type", "revolute");

        ControlSurface cs;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {}

        uadf_model.control_surfaces[joint_name] = cs;
      }
    }
  }

  // XACROを出力
  const auto xacro_text = xml::xmlDocumentToString(&uadf_doc_cp);

  // XACROを解析
  std::string urdf_text;
  if (!ros2::parseXacroFromText(xacro_text, urdf_text)) {
    return false;
  }

  // URDFを解析
  uadf_model.urdf = urdf::parseURDF(urdf_text);

  return true;
}

bool parseFromText(const std::string& uadf_text, Model& uadf_model)
{
  tinyxml2::XMLDocument uadf_doc;
  if (uadf_doc.Parse(uadf_text.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to parse UADF document." << std::endl;
    return false;
  }

  return parseFromXml(&uadf_doc, uadf_model);
}

bool parseFromPath(const std::string& uadf_path, Model& uadf_model)
{
  std::string uadf_text;
  if (!str::readText(uadf_path, uadf_text)) {
    return false;
  }

  return parseFromText(uadf_text, uadf_model);
}
}  // namespace uadf
