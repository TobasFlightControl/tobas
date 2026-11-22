// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

namespace tobas::wpa
{
enum KeyMgmt
{
  NONE,
  WPA_PSK,  ///< WPA2-Personal
  SAE,      ///< WPA3-Personal
};

std::string tokenFromEnum(KeyMgmt _key_mgmt);
bool enumFromToken(const std::string& _token, KeyMgmt& _dst);
}  // namespace tobas::wpa
