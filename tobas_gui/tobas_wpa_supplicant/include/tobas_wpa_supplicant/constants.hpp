// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace wpa
{
static constexpr char kCountryPrefix[] = "country=";
static constexpr char kCtrlInterfacePrefix[] = "ctrl_interface=";
static constexpr char kUpdateConfigPrefix[] = "update_config=";

static constexpr char kStartNetworkBlock[] = "network={";
static constexpr char kStopNetworkBlock[] = "}";
static constexpr char kSsidPrefix[] = "ssid=";
static constexpr char kPskPrefix[] = "psk=";
static constexpr char kPriorityPrefix[] = "priority=";
static constexpr char kScanSsidPrefix[] = "scan_ssid=";
static constexpr char kKeyMgmtPrefix[] = "key_mgmt=";
static constexpr char kSaePasswordPrefix[] = "sae_password=";
static constexpr char kIeee80211wPrefix[] = "ieee80211w=";
}  // namespace wpa
}  // namespace tobas
