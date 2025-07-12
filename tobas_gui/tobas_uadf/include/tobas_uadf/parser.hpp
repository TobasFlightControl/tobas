#pragma once

#include <tinyxml2.h>

#include "./model.hpp"

namespace uadf
{
bool parseFromXml(const tinyxml2::XMLDocument* uadf_doc, Model& uadf_model);
bool parseFromText(const std::string& uadf_text, Model& uadf_model);
bool parseFromPath(const std::string& uadf_path, Model& uadf_model);
}  // namespace uadf
