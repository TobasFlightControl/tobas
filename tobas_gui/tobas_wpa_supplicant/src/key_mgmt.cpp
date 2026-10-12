// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_wpa_supplicant/key_mgmt.hpp"

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

std::string tokenFromEnum(KeyMgmt _key_mgmt)
{
  switch (_key_mgmt) {
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

bool enumFromToken(const std::string& _token, KeyMgmt& _dst)
{
  if (_token == kNoneToken) {
    _dst = KeyMgmt::NONE;
    return true;
  }
  else if (_token == kWpaPskToken) {
    _dst = KeyMgmt::WPA_PSK;
    return true;
  }
  else if (_token == kSaeToken) {
    _dst = KeyMgmt::SAE;
    return true;
  }
  else {
    return false;
  }
}

std::string labelFromEnum(KeyMgmt _key_mgmt)
{
  switch (_key_mgmt) {
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

bool enumFromLabel(const std::string& _label, KeyMgmt& _dst)
{
  if (_label == kNoneLabel) {
    _dst = KeyMgmt::NONE;
    return true;
  }
  else if (_label == kWpaPskLabel) {
    _dst = KeyMgmt::WPA_PSK;
    return true;
  }
  else if (_label == kSaeLabel) {
    _dst = KeyMgmt::SAE;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace wpa
}  // namespace tobas
