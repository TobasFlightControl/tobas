#include "tobas_uadf/parser.hpp"

#include <iostream>

#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

#include <tobas_ros2_tools/xacro.hpp>

namespace uadf
{
bool parseFromText(const std::string& uadf_text, Model& uadf_model)
{
  // モデルを初期化
  uadf_model.clear();

  // XMLを読み込む
  tinyxml2::XMLDocument doc;
  if (doc.Parse(uadf_text.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to parse XML document." << std::endl;
    return false;
  }

  // ルート要素を取得
  const auto robot = doc.RootElement();
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
        child->SetAttribute("type", "continuous");
      }
      else if (strcmp(joint_type, "tilt") == 0) {
        TiltJoint tilt;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {}

        uadf_model.tilts[joint_name] = tilt;
        child->SetAttribute("type", "revolute");
      }
      else if (strcmp(joint_type, "cs") == 0) {
        ControlSurface cs;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {}

        uadf_model.control_surfaces[joint_name] = cs;
        child->SetAttribute("type", "revolute");
      }
    }
  }

  // XACROを出力
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  const auto xacro_text = printer.CStr();

  // XACROを解析
  std::string urdf_text;
  if (!ros2::parseXacroFromText(xacro_text, urdf_text)) {
    return false;
  }

  // URDFを解析
  uadf_model.urdf = urdf::parseURDF(urdf_text);

  return true;
}
}  // namespace uadf
