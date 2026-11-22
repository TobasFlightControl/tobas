// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_bootmedia_config/wifi_client/key_mgmt.hpp"

#include <stdexcept>

namespace tobas::gui::bm
{
QString keyMgmtLabel(wpa::KeyMgmt _key_mgmt)
{
  switch (_key_mgmt) {
    case wpa::KeyMgmt::NONE:
      return "None";
    case wpa::KeyMgmt::WPA_PSK:
      return "WPA2-Personal";
    case wpa::KeyMgmt::SAE:
      return "WPA3-Personal";
    default:
      throw std::invalid_argument("Invalid Wi-Fi authentication method.");
  }
}
}  // namespace tobas::gui::bm
