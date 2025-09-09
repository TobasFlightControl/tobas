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
static constexpr char kSSIDPrefix[] = "ssid=";
static constexpr char kPSKPrefix[] = "psk=";
static constexpr char kKeyMgmtPrefix[] = "key_mgmt=";
static constexpr char kPriorityPrefix[] = "priority=";
}  // namespace wpa
}  // namespace tobas
