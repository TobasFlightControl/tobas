#pragma once

namespace wpa
{
enum key_mgmt_t
{
  WPA_PSK,
  WPA_EAP,
};

namespace key_mgmt
{
static constexpr char WPA_PSK[] = "WPA-PSK";
static constexpr char WPA_EAP[] = "WPA-EAP";
}  // namespace key_mgmt
}  // namespace wpa
