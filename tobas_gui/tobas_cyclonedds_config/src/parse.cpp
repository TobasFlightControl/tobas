#include "tobas_cyclonedds_config/parse.hpp"

#include <iostream>

#include <tinyxml2.h>

#include "tobas_cyclonedds_config/constants.hpp"

namespace tobas
{
namespace cyclonedds
{
bool parseFromText(const std::string& text, Data& dst)
{
  // Clear data
  dst.interfaces.clear();

  // Parse XML
  tinyxml2::XMLDocument doc;
  if (doc.Parse(text.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to parse XML." << std::endl;
    return false;
  }

  const auto root = doc.RootElement();
  if (strcmp(root->Name(), elem::kCycloneDDS) != 0) {
    std::cerr << "The root element must be \"" << elem::kCycloneDDS << "\"." << std::endl;
    return false;
  }

  const auto domain = root->FirstChildElement(elem::kDomain);
  if (!domain) {
    return true;
  }

  const auto general = domain->FirstChildElement(elem::kGeneral);
  if (!general) {
    return true;
  }

  const auto interfaces = general->FirstChildElement(elem::kInterfaces);
  if (!interfaces) {
    return true;
  }

  for (auto nif = interfaces->FirstChildElement(elem::kNIF); nif; nif = nif->NextSiblingElement(elem::kNIF)) {
    const auto name = nif->Attribute(attr::kName);
    if (!name) {
      continue;
    }
    dst.interfaces.emplace_back(name);
  }

  return true;
}
}  // namespace cyclonedds
}  // namespace tobas
