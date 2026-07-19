// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_cyclonedds_config/export.hpp"

#include <tobas_xml_tools/core.hpp>

#include "tobas_cyclonedds_config/constants.hpp"

namespace tobas
{
namespace cyclonedds
{
namespace
{
const char* boolToText(bool v)
{
  return v ? "true" : "false";
}
}  // namespace

std::string exportText(const Data& src)
{
  tinyxml2::XMLDocument doc;

  const auto e_root = doc.NewElement(elem::kCycloneDDS);
  doc.InsertEndChild(e_root);

  const auto e_domain = e_root->InsertNewChildElement(elem::kDomain);

  {
    const auto e_general = e_domain->InsertNewChildElement(elem::kGeneral);

    {
      const auto e_ifaces = e_general->InsertNewChildElement(elem::kInterfaces);
      for (const auto& nif : src.interfaces) {
        if (nif.name.empty()) {
          continue;
        }
        const auto e_nif = e_ifaces->InsertNewChildElement(elem::kNIF);
        e_nif->SetAttribute(attr::kName, nif.name.c_str());
        e_nif->SetAttribute(attr::kPriority, nif.priority);
      }
    }

    e_general->InsertNewChildElement(elem::kRedundantNetworking)->SetText(boolToText(src.redundant_networking));
  }

  {
    const auto e_shared_memory = e_domain->InsertNewChildElement(elem::kSharedMemory);

    const auto e_enable = e_shared_memory->InsertNewChildElement(elem::kEnable);
    e_enable->SetText(boolToText(src.shared_memory.enable));

    const auto e_log_level = e_shared_memory->InsertNewChildElement(elem::kLogLevel);
    switch (src.shared_memory.log_level) {
      case SharedMemory::kVerbose:
        e_log_level->SetText("verbose");
        break;
      case SharedMemory::kDebug:
        e_log_level->SetText("debug");
        break;
      case SharedMemory::kInfo:
        e_log_level->SetText("info");
        break;
      case SharedMemory::kWarn:
        e_log_level->SetText("warn");
        break;
      case SharedMemory::kError:
        e_log_level->SetText("error");
        break;
      case SharedMemory::kFatal:
        e_log_level->SetText("fatal");
        break;
      case SharedMemory::kOff:
        e_log_level->SetText("off");
        break;
      default:
        throw;
    }
  }

  return xml::xmlDocumentToString(&doc);
}
}  // namespace cyclonedds
}  // namespace tobas
