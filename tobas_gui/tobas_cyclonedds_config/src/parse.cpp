// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_cyclonedds_config/parse.hpp"

#include <iostream>

#include <tobas_xml_tools/core.hpp>

#include "tobas_cyclonedds_config/constants.hpp"

namespace tobas
{
namespace cyclonedds
{
namespace
{
bool textToBool(const char* text)
{
  return std::strcmp(text, "true") == 0;
}
}  // namespace

bool parseFromText(const std::string& text, Data& dst)
{
  // Clear data.
  dst.interfaces.clear();

  // Parse XML.
  tinyxml2::XMLDocument doc;
  if (doc.Parse(text.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to parse XML." << std::endl;
    return false;
  }

  const auto e_root = doc.RootElement();
  if (std::strcmp(e_root->Name(), elem::kCycloneDDS) != 0) {
    std::cerr << "The root element must be \"" << elem::kCycloneDDS << "\"." << std::endl;
    return false;
  }

  const auto e_domain = e_root->FirstChildElement(elem::kDomain);
  if (e_domain) {
    const auto e_general = e_domain->FirstChildElement(elem::kGeneral);
    if (e_general) {
      const auto e_ifaces = e_general->FirstChildElement(elem::kInterfaces);
      if (e_ifaces) {
        for (auto e_nif = e_ifaces->FirstChildElement(elem::kNIF); e_nif;
             e_nif = e_nif->NextSiblingElement(elem::kNIF)) {
          const auto name = e_nif->Attribute(attr::kName);
          if (!name) {
            continue;
          }
          tobas::cyclonedds::NetworkInterface nif;
          nif.name = name;
          nif.priority = xml::getAttribute<int>(e_nif, attr::kPriority);
          nif.multicast = xml::getAttribute<bool>(e_nif, attr::kMulticast);
          dst.interfaces.push_back(nif);
        }
      }

      const auto e_redundant_networking = e_general->FirstChildElement(elem::kRedundantNetworking);
      if (e_redundant_networking) {
        dst.redundant_networking = textToBool(e_redundant_networking->GetText());
      }
    }

    const auto e_shared_memory = e_domain->FirstChildElement(elem::kSharedMemory);
    if (e_shared_memory) {
      const auto e_enable = e_shared_memory->FirstChildElement(elem::kEnable);
      if (e_enable) {
        dst.shared_memory.enable = textToBool(e_enable->GetText());
      }
      const auto log_level = e_shared_memory->FirstChildElement(elem::kLogLevel);
      if (log_level) {
        const auto log_level_text = log_level->GetText();
        if (std::strcmp(log_level_text, "verbose") == 0) {
          dst.shared_memory.log_level = SharedMemory::kVerbose;
        }
        else if (std::strcmp(log_level_text, "debug") == 0) {
          dst.shared_memory.log_level = SharedMemory::kDebug;
        }
        else if (std::strcmp(log_level_text, "info") == 0) {
          dst.shared_memory.log_level = SharedMemory::kInfo;
        }
        else if (std::strcmp(log_level_text, "warn") == 0) {
          dst.shared_memory.log_level = SharedMemory::kWarn;
        }
        else if (std::strcmp(log_level_text, "error") == 0) {
          dst.shared_memory.log_level = SharedMemory::kError;
        }
        else if (std::strcmp(log_level_text, "fatal") == 0) {
          dst.shared_memory.log_level = SharedMemory::kFatal;
        }
        else if (std::strcmp(log_level_text, "off") == 0) {
          dst.shared_memory.log_level = SharedMemory::kOff;
        }
        else {
          std::cerr << "Invalid shared memory log level: " << log_level_text << std::endl;
        }
      }
    }
  }

  return true;
}
}  // namespace cyclonedds
}  // namespace tobas
