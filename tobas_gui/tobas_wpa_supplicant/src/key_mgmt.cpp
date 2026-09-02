// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_wpa_supplicant/key_mgmt.hpp"

#include <iostream>

namespace tobas
{
namespace wpa
{
namespace
{
constexpr char kNoneToken[] = "NONE";
constexpr char kWpaPskToken[] = "WPA-PSK";
constexpr char kSaeToken[] = "SAE";

constexpr char kNoneLabel[] = "None";
constexpr char kWpaPskLabel[] = "WPA2-Personal";
constexpr char kSaeLabel[] = "WPA3-Personal";
}  // namespace

std::string tokenFromEnum(KeyMgmt key_mgmt)
{
  switch (key_mgmt) {
    case KeyMgmt::NONE:
      return kNoneToken;
    case KeyMgmt::WPA_PSK:
      return kWpaPskToken;
    case KeyMgmt::SAE:
      return kSaeToken;
    default:
      throw;
  }
}

bool enumFromToken(const std::string& token, KeyMgmt& dst)
{
  if (token == kNoneToken) {
    dst = KeyMgmt::NONE;
    return true;
  }
  else if (token == kWpaPskToken) {
    dst = KeyMgmt::WPA_PSK;
    return true;
  }
  else if (token == kSaeToken) {
    dst = KeyMgmt::SAE;
    return true;
  }
  else {
    std::cerr << "Key management \"" << token << "\" is not supported." << std::endl;
    return false;
  }
}

std::string labelFromEnum(KeyMgmt key_mgmt)
{
  switch (key_mgmt) {
    case KeyMgmt::NONE:
      return kNoneLabel;
    case KeyMgmt::WPA_PSK:
      return kWpaPskLabel;
    case KeyMgmt::SAE:
      return kSaeLabel;
    default:
      throw;
  }
}

bool enumFromLabel(const std::string& label, KeyMgmt& dst)
{
  if (label == kNoneLabel) {
    dst = KeyMgmt::NONE;
    return true;
  }
  else if (label == kWpaPskLabel) {
    dst = KeyMgmt::WPA_PSK;
    return true;
  }
  else if (label == kSaeLabel) {
    dst = KeyMgmt::SAE;
    return true;
  }
  else {
    std::cerr << "Invalid key management: " << label << std::endl;
    return false;
  }
}
}  // namespace wpa
}  // namespace tobas
