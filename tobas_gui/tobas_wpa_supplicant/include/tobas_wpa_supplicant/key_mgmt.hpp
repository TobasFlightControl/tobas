#pragma once

namespace tobas
{
namespace wpa
{
enum KeyManagement
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
}  // namespace tobas
