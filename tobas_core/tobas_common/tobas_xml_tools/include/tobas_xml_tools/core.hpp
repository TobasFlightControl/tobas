#pragma once

#include <string>

#include <tinyxml2.h>

namespace tobas
{
namespace xml
{
std::string xmlDocumentToString(const tinyxml2::XMLDocument* doc);
}  // namespace xml
}  // namespace tobas
