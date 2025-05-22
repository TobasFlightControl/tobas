#pragma once

#include <filesystem>
#include <vector>

#include "./country_code.hpp"
#include "./network.hpp"

namespace wpa
{
class WpaSupplicantParser
{
  static constexpr country_code_t kDefaultCountry = country_code_t::JP;
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
  country_code_t country = kDefaultCountry;
  std::string ctrl_interface = kDefaultCtrlInterface;
  bool update_config = kDefaultUpdateConfig;
  std::vector<Network> networks;

  explicit WpaSupplicantParser();

  /* 設定を初期化する． */
  void clear();

  bool parseFromText(const std::string& text);
  bool parseFromFile(const std::filesystem::path& path);

  /* 設定をファイルに保存する． */
  bool write(const std::filesystem::path& path);

  /* 設定をwpa_supplicant.confのテキスト形式で返す． */
  std::string exportText() const;

private:
  static bool parseCountryCode(const std::string& src, country_code_t& dst);
  static bool parseKeyManagement(const std::string& src, key_mgmt_t& dst);

  static const char* countryCodeToString(country_code_t cc);
  static const char* keyManagementToString(key_mgmt_t key_mgmt);
};
}  // namespace wpa
