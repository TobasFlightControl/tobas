// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_cyclonedds_config/export.hpp"

#include <tobas_xml_tools/core.hpp>

#include "tobas_cyclonedds_config/constants.hpp"

namespace tobas
{
namespace cyclonedds
{
std::string exportText(const Data& src)
{
  tinyxml2::XMLDocument doc;

  const auto e_root = doc.NewElement(elem::kCycloneDDS);
  doc.InsertEndChild(e_root);

  const auto e_domain = e_root->InsertNewChildElement(elem::kDomain);
  const auto e_general = e_domain->InsertNewChildElement(elem::kGeneral);
  const auto e_ifaces = e_general->InsertNewChildElement(elem::kInterfaces);

  for (const auto& nif : src.interfaces) {
    if (nif.name.empty()) {
      continue;
    }
    const auto e_nif = e_ifaces->InsertNewChildElement(elem::kNIF);
    e_nif->SetAttribute(attr::kName, nif.name.c_str());
  }

  return xml::xmlDocumentToString(&doc);
}
}  // namespace cyclonedds
}  // namespace tobas
