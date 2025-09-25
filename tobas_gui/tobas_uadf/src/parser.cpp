#include "tobas_uadf/parser.hpp"

#include <tobas_string_tools/stream.hpp>
#include <tobas_xml_tools/core.hpp>

namespace uadf
{
Parser::Parser()
{
}

bool Parser::parseFromXml(const tinyxml2::XMLDocument* uadf_doc, Model& uadf_model)
{
  // モデルを初期化
  uadf_model.clear();

  // 編集用にXMLドキュメントをコピー
  tinyxml2::XMLDocument uadf_doc_cp;
  uadf_doc->DeepCopy(&uadf_doc_cp);

  // ルート要素を取得
  const auto robot = uadf_doc_cp.RootElement();

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
              error_msg_ = "Thrust joint \"" + std::string(joint_name) + "\" has invalid direction \"" +
                           std::string(direction) + "\". It must be \"cw\" or \"ccw\".";
              return false;
            }
          }
        }

        if (!direction_found) {
          error_msg_ = "Thrust joint \"" + std::string(joint_name) + "\" has no \"direction\" element.";
          return false;
        }

        uadf_model.thrusts[joint_name] = thrust;
      }
      else if (strcmp(joint_type, "cs") == 0) {
        child->SetAttribute("type", "revolute");

        ControlSurface cs;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {}

        uadf_model.control_surfaces[joint_name] = cs;
      }
      else if (strcmp(joint_type, "tilt") == 0) {
        child->SetAttribute("type", "revolute");

        TiltJoint tilt;

        for (auto gchild = child->FirstChildElement(); gchild; gchild = gchild->NextSiblingElement()) {}

        uadf_model.tilts[joint_name] = tilt;
      }
    }
  }

  // URDFを書き出す
  const auto urdf_text = xml::xmlDocumentToString(&uadf_doc_cp);

  // URDFを解析
  uadf_model.urdf = urdf_parser_.parseFromText(urdf_text);
  if (!uadf_model.urdf) {
    error_msg_ = urdf_parser_.errorMessage();
    return false;
  }

  return true;
}

bool Parser::parseFromText(const std::string& uadf_text, Model& uadf_model)
{
  tinyxml2::XMLDocument uadf_doc;
  if (uadf_doc.Parse(uadf_text.c_str()) != tinyxml2::XML_SUCCESS) {
    error_msg_ = uadf_doc.ErrorStr();
    return false;
  }

  return parseFromXml(&uadf_doc, uadf_model);
}

bool Parser::parseFromPath(const std::string& uadf_path, Model& uadf_model)
{
  std::string uadf_text;
  if (!str::readText(uadf_path, uadf_text)) {
    error_msg_ = "Failed to open file: " + uadf_path;
    return false;
  }

  return parseFromText(uadf_text, uadf_model);
}

const std::string& Parser::errorMessage() const
{
  return error_msg_;
}
}  // namespace uadf
