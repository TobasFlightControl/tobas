#include <iostream>
#include <fstream>

#include <tobas_string_tools/core.hpp>

#include "../include/wpa_supplicant_parser/parser.hpp"

using namespace std;
namespace fs = filesystem;

namespace wpa
{
WPASupplicantParser::WPASupplicantParser()
{
}

void WPASupplicantParser::clear()
{
  country = kDefaultCountry;
  ctrl_interface = kDefaultCtrlInterface;
  update_config = kDefaultUpdateConfig;
  networks.clear();
}

bool WPASupplicantParser::parseFromText(const string& text)
{
  clear();

  const auto lines = str::splitLines(text);
  Network network;
  bool in_network_block = false;

  for (auto line : lines)
  {
    // 行頭・行末の空白を削除
    line = str::trim(line);

    // 空行やコメント行をスキップ
    if (line.empty() || line.starts_with('#'))
      continue;

    // country
    if (line.starts_with(kCountryPrefix))
    {
      const auto country_str = line.substr(sizeof(kCountryPrefix) - 1);
      if (!parseCountryCode(country_str, country))
        return false;
      continue;
    }

    // ctrl_interface
    if (line.starts_with(kCtrlInterfacePrefix))
    {
      ctrl_interface = line.substr(sizeof(kCtrlInterfacePrefix) - 1);
      continue;
    }

    // update_config
    if (line.starts_with(kUpdateConfigPrefix))
    {
      update_config = (line.substr(sizeof(kUpdateConfigPrefix) - 1) == "1");
      continue;
    }

    // ネットワークブロックの開始
    if (line == kStartNetworkBlock)
    {
      network = Network();
      in_network_block = true;
      continue;
    }

    // ネットワークブロックの終了
    if (line == kStopNetworkBlock)
    {
      if (!in_network_block)
      {
        cerr << "Unexpected closing bracket." << endl;
        return false;
      }
      networks.push_back(network);
      in_network_block = false;
      continue;
    }

    // ssid
    if (line.starts_with(kSSIDPrefix))
    {
      if (!in_network_block)
      {
        cerr << "SSID setting found outside network block." << endl;
        return false;
      }
      network.ssid = str::stripQuates(line.substr(sizeof(kSSIDPrefix) - 1));
      continue;
    }

    // psk
    if (line.starts_with(kPSKPrefix))
    {
      if (!in_network_block)
      {
        cerr << "PSK setting found outside network block." << endl;
        return false;
      }
      network.psk = str::stripQuates(line.substr(sizeof(kPSKPrefix) - 1));
      continue;
    }

    // key_mgmt
    if (line.starts_with(kKeyMgmtPrefix))
    {
      if (!in_network_block)
      {
        cerr << "Key management setting found outside network block." << endl;
        return false;
      }
      const auto key_mgmt_str = line.substr(sizeof(kKeyMgmtPrefix) - 1);
      if (!parseKeyManagement(key_mgmt_str, network.key_mgmt))
        return false;
      continue;
    }

    // priority
    if (line.starts_with(kPriorityPrefix))
    {
      if (!in_network_block)
      {
        cerr << "Priority setting found outside network block." << endl;
        return false;
      }
      network.priority = stoi(line.substr(sizeof(kPriorityPrefix) - 1));
      continue;
    }
  }

  return true;
}

bool WPASupplicantParser::parseFromFile(const fs::path& path)
{
  std::ifstream file(path);
  if (!file)
  {
    cerr << "Failed to open file: " << path << endl;
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();  // ファイル全体を文字列に読み込む
  file.close();

  const auto text = buffer.str();
  return parseFromText(text);  // テキストからパース
}

bool WPASupplicantParser::write(const fs::path& path)
{
  std::ofstream file(path);
  if (!file)
  {
    cerr << "Failed to open file for writing: " << path << endl;
    return false;
  }

  file << text();  // テキスト形式で書き込む
  file.close();

  return true;
}

string WPASupplicantParser::text() const
{
  std::ostringstream oss;

  // Country
  oss << kCountryPrefix << countryCodeToString(country) << "\n";

  // ctrl_interface
  oss << kCtrlInterfacePrefix << ctrl_interface << "\n";

  // update_config
  oss << kUpdateConfigPrefix << (update_config ? "1" : "0") << "\n";

  // Networks
  for (const auto& network : networks)
  {
    oss << "\n" << kStartNetworkBlock << "\n";
    oss << "\t" << kSSIDPrefix << "\"" << network.ssid << "\"\n";
    oss << "\t" << kPSKPrefix << "\"" << network.psk << "\"\n";
    oss << "\t" << kKeyMgmtPrefix << keyManagementToString(network.key_mgmt) << "\n";
    oss << "\t" << kPriorityPrefix << network.priority << "\n";
    oss << kStopNetworkBlock << "\n";
  }

  return oss.str();
}

bool WPASupplicantParser::parseCountryCode(const string& src, country_code_t& dst)
{
  // TODO: 全ての国について書く

  if (src == country_code::China)
    dst = country_code_t::CN;
  else if (src == country_code::India)
    dst = country_code_t::IN;
  else if (src == country_code::Japan)
    dst = country_code_t::JP;
  else if (src == country_code::United_Kingdom)
    dst = country_code_t::GB;
  else if (src == country_code::United_State)
    dst = country_code_t::US;
  else
  {
    cerr << "Invalid country code: " << src << endl;
    return false;
  }

  return true;
}

bool WPASupplicantParser::parseKeyManagement(const string& src, key_mgmt_t& dst)
{
  if (src == key_mgmt::WPA_PSK)
    dst = key_mgmt_t::WPA_PSK;
  else if (src == key_mgmt::WPA_EAP)
    dst = key_mgmt_t::WPA_EAP;
  else
  {
    cerr << "Invalid key management method: " << src << endl;
    return false;
  }

  return true;
}

const char* WPASupplicantParser::countryCodeToString(country_code_t cc)
{
  // TODO: 全ての国について書く

  switch (cc)
  {
    case country_code_t::CH:
      return country_code::China;
    case country_code_t::IN:
      return country_code::India;
    case country_code_t::JP:
      return country_code::Japan;
    case country_code_t::GB:
      return country_code::United_Kingdom;
    case country_code_t::US:
      return country_code::United_State;
    default:
      throw;
  }
}

const char* WPASupplicantParser::keyManagementToString(key_mgmt_t key_mgmt)
{
  switch (key_mgmt)
  {
    case key_mgmt_t::WPA_PSK:
      return key_mgmt::WPA_PSK;
    case key_mgmt_t::WPA_EAP:
      return key_mgmt::WPA_EAP;
    default:
      throw;
  }
}
}  // namespace wpa
