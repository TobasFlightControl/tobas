// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas
{
namespace wpa
{
enum KeyMgmt
{
  NONE,
  WPA_PSK,  // WPA2-Personal
  SAE,      // WPA3-Personal
};

std::string tokenFromEnum(KeyMgmt key_mgmt);
bool enumFromToken(const std::string& token, KeyMgmt& dst);

std::string labelFromEnum(KeyMgmt key_mgmt);
bool enumFromLabel(const std::string& label, KeyMgmt& dst);
}  // namespace wpa
}  // namespace tobas
