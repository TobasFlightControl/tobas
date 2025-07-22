#pragma once

#include <filesystem>
#include <vector>

#include "./country_code.hpp"
#include "./network.hpp"

namespace wpa
{
class WpaSupplicantParser
{
  static constexpr CountryCode kDefaultCountry = CountryCode::JP;
  static constexpr char kDefaultCtrlInterface[] = "DIR=/var/run/wpa_supplicant GROUP=netdev";
  static constexpr bool kDefaultUpdateConfig = true;

  static constexpr char kCountryPrefix[] = "country=";
  static constexpr char kCtrlInterfacePrefix[] = "ctrl_interface=";
  static constexpr char kUpdateConfigPrefix[] = "update_config=";

  static constexpr char kStartNetworkBlock[] = "network={";
  static constexpr char kStopNetworkBlock[] = "}";
  static constexpr char kSSIDPrefix[] = "ssid=";
  static constexpr char kPSKPrefix[] = "psk=";
  static constexpr char kKeyMgmtPrefix[] = "key_mgmt=";
  static constexpr char kPriorityPrefix[] = "priority=";

public:
  CountryCode country = kDefaultCountry;
  std::string ctrl_interface = kDefaultCtrlInterface;
  bool update_config = kDefaultUpdateConfig;
  std::vector<Network> networks;

  explicit WpaSupplicantParser();

  /* 設定を初期化する． */
  void clear();

  bool parseFromText(const std::string& text);
  bool parseFromPath(const std::filesystem::path& path);

  /* 設定をファイルに保存する． */
  bool write(const std::filesystem::path& path);

  /* 設定をwpa_supplicant.confのテキスト形式で返す． */
  std::string exportText() const;

private:
  static bool parseCountryCode(const std::string& src, CountryCode& dst);
  static bool parseKeyManagement(const std::string& src, KeyManagement& dst);

  static const char* countryCodeToString(CountryCode cc);
  static const char* keyManagementToString(KeyManagement key_mgmt);
};
}  // namespace wpa
