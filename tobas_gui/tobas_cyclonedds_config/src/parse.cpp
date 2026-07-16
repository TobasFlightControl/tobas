// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

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
  // Clear data.
  dst.interfaces.clear();

  // Parse XML.
  tinyxml2::XMLDocument doc;
  if (doc.Parse(text.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cerr << "Failed to parse XML." << std::endl;
    return false;
  }

  const auto root = doc.RootElement();
  if (std::strcmp(root->Name(), elem::kCycloneDDS) != 0) {
    std::cerr << "The root element must be \"" << elem::kCycloneDDS << "\"." << std::endl;
    return false;
  }

  const auto domain = root->FirstChildElement(elem::kDomain);
  if (domain) {
    const auto general = domain->FirstChildElement(elem::kGeneral);
    if (general) {
      const auto interfaces = general->FirstChildElement(elem::kInterfaces);
      if (interfaces) {
        for (auto nif = interfaces->FirstChildElement(elem::kNIF); nif; nif = nif->NextSiblingElement(elem::kNIF)) {
          const auto name = nif->Attribute(attr::kName);
          if (!name) {
            continue;
          }
          dst.interfaces.emplace_back(name);
        }
      }
    }

    const auto shared_memory = domain->FirstChildElement(elem::kSharedMemory);
    if (shared_memory) {
      const auto enable = shared_memory->FirstChildElement(elem::kEnable);
      if (enable) {
        dst.shared_memory.enable = (std::strcmp(enable->GetText(), "true") == 0);
      }
      const auto log_level = shared_memory->FirstChildElement(elem::kLogLevel);
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
