// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_cyclonedds_config/parse.hpp"

#include <cstring>
#include <format>

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
  return text && std::strcmp(text, "true") == 0;
}
}  // namespace

std::expected<Data, std::string> parseFromText(const std::string& text)
{
  Data data;

  // Parse XML.
  tinyxml2::XMLDocument doc;
  if (doc.Parse(text.c_str()) != tinyxml2::XML_SUCCESS) {
    return std::unexpected(std::format("Failed to parse XML: {}", doc.ErrorStr()));
  }

  const auto e_root = doc.RootElement();
  if (!e_root || std::strcmp(e_root->Name(), elem::kCycloneDDS) != 0) {
    return std::unexpected(std::format("The root element must be '{}'.", elem::kCycloneDDS));
  }

  const auto e_domain = e_root->FirstChildElement(elem::kDomain);
  if (e_domain) {
    const auto e_general = e_domain->FirstChildElement(elem::kGeneral);
    if (e_general) {
      const auto e_ifaces = e_general->FirstChildElement(elem::kInterfaces);
      if (e_ifaces) {
        for (auto e_nic = e_ifaces->FirstChildElement(elem::kNetworkIface); e_nic;
             e_nic = e_nic->NextSiblingElement(elem::kNetworkIface)) {
          const auto name = e_nic->Attribute(attr::kName);
          if (!name) {
            continue;
          }
          tobas::cyclonedds::NetworkInterface nic;
          nic.name = name;
          nic.priority = xml::getAttribute<int>(e_nic, attr::kPriority);
          nic.multicast = xml::getAttribute<bool>(e_nic, attr::kMulticast);
          data.interfaces.push_back(nic);
        }
      }

      const auto e_redundant_networking = e_general->FirstChildElement(elem::kRedundantNetworking);
      if (e_redundant_networking) {
        data.redundant_networking = textToBool(e_redundant_networking->GetText());
      }
    }

    const auto e_shared_memory = e_domain->FirstChildElement(elem::kSharedMemory);
    if (e_shared_memory) {
      const auto e_enable = e_shared_memory->FirstChildElement(elem::kEnable);
      if (e_enable) {
        data.shared_memory.enable = textToBool(e_enable->GetText());
      }
      const auto log_level = e_shared_memory->FirstChildElement(elem::kLogLevel);
      if (log_level) {
        const auto log_level_text = log_level->GetText();
        if (!log_level_text) {
          return std::unexpected("Shared memory log level is empty.");
        }
        if (std::strcmp(log_level_text, "verbose") == 0) {
          data.shared_memory.log_level = SharedMemory::kVerbose;
        }
        else if (std::strcmp(log_level_text, "debug") == 0) {
          data.shared_memory.log_level = SharedMemory::kDebug;
        }
        else if (std::strcmp(log_level_text, "info") == 0) {
          data.shared_memory.log_level = SharedMemory::kInfo;
        }
        else if (std::strcmp(log_level_text, "warn") == 0) {
          data.shared_memory.log_level = SharedMemory::kWarn;
        }
        else if (std::strcmp(log_level_text, "error") == 0) {
          data.shared_memory.log_level = SharedMemory::kError;
        }
        else if (std::strcmp(log_level_text, "fatal") == 0) {
          data.shared_memory.log_level = SharedMemory::kFatal;
        }
        else if (std::strcmp(log_level_text, "off") == 0) {
          data.shared_memory.log_level = SharedMemory::kOff;
        }
        else {
          return std::unexpected(std::format("Invalid shared memory log level: {}", log_level_text));
        }
      }
    }
  }

  return data;
}
}  // namespace cyclonedds
}  // namespace tobas
