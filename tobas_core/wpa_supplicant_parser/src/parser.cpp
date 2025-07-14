#include "wpa_supplicant_parser/parser.hpp"

#include <iostream>

#include <tobas_string_tools/core.hpp>
#include <tobas_string_tools/stream.hpp>

using namespace std;
namespace fs = filesystem;

namespace wpa
{
WpaSupplicantParser::WpaSupplicantParser()
{
}

void WpaSupplicantParser::clear()
{
  country = kDefaultCountry;
  ctrl_interface = kDefaultCtrlInterface;
  update_config = kDefaultUpdateConfig;
  networks.clear();
}

bool WpaSupplicantParser::parseFromText(const string& text)
{
  clear();

  const auto lines = str::splitLines(text);
  Network network;
  bool in_network_block = false;

  for (auto line : lines) {
    // 行頭・行末の空白を削除
    line = str::trim(line);

    // 空行やコメント行をスキップ
    if (line.empty() || line.starts_with('#')) {
      continue;
    }

    // country
    if (line.starts_with(kCountryPrefix)) {
      const auto country_str = line.substr(sizeof(kCountryPrefix) - 1);
      if (!parseCountryCode(country_str, country)) {
        return false;
      }
      continue;
    }

    // ctrl_interface
    if (line.starts_with(kCtrlInterfacePrefix)) {
      ctrl_interface = line.substr(sizeof(kCtrlInterfacePrefix) - 1);
      continue;
    }

    // update_config
    if (line.starts_with(kUpdateConfigPrefix)) {
      update_config = (line.substr(sizeof(kUpdateConfigPrefix) - 1) == "1");
      continue;
    }

    // ネットワークブロックの開始
    if (line == kStartNetworkBlock) {
      network = Network();
      in_network_block = true;
      continue;
    }

    // ネットワークブロックの終了
    if (line == kStopNetworkBlock) {
      if (!in_network_block) {
        cerr << "Unexpected closing bracket." << endl;
        return false;
      }
      networks.push_back(network);
      in_network_block = false;
      continue;
    }

    // ssid
    if (line.starts_with(kSSIDPrefix)) {
      if (!in_network_block) {
        cerr << "SSID setting found outside network block." << endl;
        return false;
      }
      network.ssid = str::stripQuates(line.substr(sizeof(kSSIDPrefix) - 1));
      continue;
    }

    // psk
    if (line.starts_with(kPSKPrefix)) {
      if (!in_network_block) {
        cerr << "PSK setting found outside network block." << endl;
        return false;
      }
      network.psk = str::stripQuates(line.substr(sizeof(kPSKPrefix) - 1));
      continue;
    }

    // key_mgmt
    if (line.starts_with(kKeyMgmtPrefix)) {
      if (!in_network_block) {
        cerr << "Key management setting found outside network block." << endl;
        return false;
      }
      const auto key_mgmt_str = line.substr(sizeof(kKeyMgmtPrefix) - 1);
      if (!parseKeyManagement(key_mgmt_str, network.key_mgmt)) {
        return false;
      }
      continue;
    }

    // priority
    if (line.starts_with(kPriorityPrefix)) {
      if (!in_network_block) {
        cerr << "Priority setting found outside network block." << endl;
        return false;
      }
      network.priority = stoi(line.substr(sizeof(kPriorityPrefix) - 1));
      continue;
    }
  }

  return true;
}

bool WpaSupplicantParser::parseFromPath(const fs::path& path)
{
  string text;
  if (!str::readText(path, text)) {
    return false;
  }

  return parseFromText(text);
}

bool WpaSupplicantParser::write(const fs::path& path)
{
  return str::writeText(path, exportText());
}

string WpaSupplicantParser::exportText() const
{
  ostringstream oss;

  // Country
  oss << kCountryPrefix << countryCodeToString(country) << "\n";

  // ctrl_interface
  oss << kCtrlInterfacePrefix << ctrl_interface << "\n";

  // update_config
  oss << kUpdateConfigPrefix << (update_config ? "1" : "0") << "\n";

  // Networks
  for (const auto& network : networks) {
    oss << "\n" << kStartNetworkBlock << "\n";
    oss << "\t" << kSSIDPrefix << "\"" << network.ssid << "\"\n";
    oss << "\t" << kPSKPrefix << "\"" << network.psk << "\"\n";
    oss << "\t" << kKeyMgmtPrefix << keyManagementToString(network.key_mgmt) << "\n";
    oss << "\t" << kPriorityPrefix << network.priority << "\n";
    oss << kStopNetworkBlock << "\n";
  }

  return oss.str();
}

bool WpaSupplicantParser::parseCountryCode(const string& src, country_code_t& dst)
{
  if (src == country_code::Afghanistan) {
    dst = country_code_t::AF;
    return true;
  }
  else if (src == country_code::Albania) {
    dst = country_code_t::AL;
    return true;
  }
  else if (src == country_code::Algeria) {
    dst = country_code_t::DZ;
    return true;
  }
  else if (src == country_code::American_Samoa) {
    dst = country_code_t::AS;
    return true;
  }
  else if (src == country_code::Andorra) {
    dst = country_code_t::AD;
    return true;
  }
  else if (src == country_code::Angola) {
    dst = country_code_t::AO;
    return true;
  }
  else if (src == country_code::Anguilla) {
    dst = country_code_t::AI;
    return true;
  }
  else if (src == country_code::Antarctica) {
    dst = country_code_t::AQ;
    return true;
  }
  else if (src == country_code::Antigua_and_Barbuda) {
    dst = country_code_t::AG;
    return true;
  }
  else if (src == country_code::Argentina) {
    dst = country_code_t::AR;
    return true;
  }
  else if (src == country_code::Armenia) {
    dst = country_code_t::AM;
    return true;
  }
  else if (src == country_code::Aruba) {
    dst = country_code_t::AW;
    return true;
  }
  else if (src == country_code::Australia) {
    dst = country_code_t::AU;
    return true;
  }
  else if (src == country_code::Austria) {
    dst = country_code_t::AT;
    return true;
  }
  else if (src == country_code::Azerbaijan) {
    dst = country_code_t::AZ;
    return true;
  }
  else if (src == country_code::Bahamas) {
    dst = country_code_t::BS;
    return true;
  }
  else if (src == country_code::Bahrain) {
    dst = country_code_t::BH;
    return true;
  }
  else if (src == country_code::Bangladesh) {
    dst = country_code_t::BD;
    return true;
  }
  else if (src == country_code::Barbados) {
    dst = country_code_t::BB;
    return true;
  }
  else if (src == country_code::Belarus) {
    dst = country_code_t::BY;
    return true;
  }
  else if (src == country_code::Belgium) {
    dst = country_code_t::BE;
    return true;
  }
  else if (src == country_code::Belize) {
    dst = country_code_t::BZ;
    return true;
  }
  else if (src == country_code::Benin) {
    dst = country_code_t::BJ;
    return true;
  }
  else if (src == country_code::Bermuda) {
    dst = country_code_t::BM;
    return true;
  }
  else if (src == country_code::Bhutan) {
    dst = country_code_t::BT;
    return true;
  }
  else if (src == country_code::Bolivia) {
    dst = country_code_t::BO;
    return true;
  }
  else if (src == country_code::Bosnia_and_Herzegovina) {
    dst = country_code_t::BA;
    return true;
  }
  else if (src == country_code::Botswana) {
    dst = country_code_t::BW;
    return true;
  }
  else if (src == country_code::Brazil) {
    dst = country_code_t::BR;
    return true;
  }
  else if (src == country_code::British_Indian_OceanTerritory) {
    dst = country_code_t::IO;
    return true;
  }
  else if (src == country_code::British_Virgin_Islands) {
    dst = country_code_t::VG;
    return true;
  }
  else if (src == country_code::Brunei) {
    dst = country_code_t::BN;
    return true;
  }
  else if (src == country_code::Bulgaria) {
    dst = country_code_t::BG;
    return true;
  }
  else if (src == country_code::Burkina_Faso) {
    dst = country_code_t::BF;
    return true;
  }
  else if (src == country_code::Burundi) {
    dst = country_code_t::BI;
    return true;
  }
  else if (src == country_code::Cambodia) {
    dst = country_code_t::KH;
    return true;
  }
  else if (src == country_code::Cameroon) {
    dst = country_code_t::CM;
    return true;
  }
  else if (src == country_code::Canad) {
    dst = country_code_t::CA;
    return true;
  }
  else if (src == country_code::Cape_Verde) {
    dst = country_code_t::CV;
    return true;
  }
  else if (src == country_code::Cayman_Islands) {
    dst = country_code_t::KY;
    return true;
  }
  else if (src == country_code::Central_African_Republic) {
    dst = country_code_t::CF;
    return true;
  }
  else if (src == country_code::Chad) {
    dst = country_code_t::TD;
    return true;
  }
  else if (src == country_code::Chile) {
    dst = country_code_t::CL;
    return true;
  }
  else if (src == country_code::China) {
    dst = country_code_t::CN;
    return true;
  }
  else if (src == country_code::Christmas_Island) {
    dst = country_code_t::CX;
    return true;
  }
  else if (src == country_code::Cocos_Islands) {
    dst = country_code_t::CC;
    return true;
  }
  else if (src == country_code::Colombia) {
    dst = country_code_t::CO;
    return true;
  }
  else if (src == country_code::Comoros) {
    dst = country_code_t::KM;
    return true;
  }
  else if (src == country_code::Cook_Islands) {
    dst = country_code_t::CK;
    return true;
  }
  else if (src == country_code::Costa_Rica) {
    dst = country_code_t::CR;
    return true;
  }
  else if (src == country_code::Croatia) {
    dst = country_code_t::HR;
    return true;
  }
  else if (src == country_code::Cuba) {
    dst = country_code_t::CU;
    return true;
  }
  else if (src == country_code::Curacao) {
    dst = country_code_t::CW;
    return true;
  }
  else if (src == country_code::Cyprus) {
    dst = country_code_t::CY;
    return true;
  }
  else if (src == country_code::Czech_Republic) {
    dst = country_code_t::CZ;
    return true;
  }
  else if (src == country_code::Democratic_Republic_of_the_Congo) {
    dst = country_code_t::CD;
    return true;
  }
  else if (src == country_code::Denmark) {
    dst = country_code_t::DK;
    return true;
  }
  else if (src == country_code::Djibouti) {
    dst = country_code_t::DJ;
    return true;
  }
  else if (src == country_code::Dominica) {
    dst = country_code_t::DM;
    return true;
  }
  else if (src == country_code::Dominican_Republic) {
    dst = country_code_t::DO;
    return true;
  }
  else if (src == country_code::East_Timor) {
    dst = country_code_t::TL;
    return true;
  }
  else if (src == country_code::Ecuador) {
    dst = country_code_t::EC;
    return true;
  }
  else if (src == country_code::Egypt) {
    dst = country_code_t::EG;
    return true;
  }
  else if (src == country_code::El_Salvador) {
    dst = country_code_t::SV;
    return true;
  }
  else if (src == country_code::Equatorial_Guinea) {
    dst = country_code_t::GQ;
    return true;
  }
  else if (src == country_code::Eritrea) {
    dst = country_code_t::ER;
    return true;
  }
  else if (src == country_code::Estonia) {
    dst = country_code_t::EE;
    return true;
  }
  else if (src == country_code::Ethiopia) {
    dst = country_code_t::ET;
    return true;
  }
  else if (src == country_code::Falkland_Islands) {
    dst = country_code_t::FK;
    return true;
  }
  else if (src == country_code::Faroe_Islands) {
    dst = country_code_t::FO;
    return true;
  }
  else if (src == country_code::Fiji) {
    dst = country_code_t::FJ;
    return true;
  }
  else if (src == country_code::Finland) {
    dst = country_code_t::FI;
    return true;
  }
  else if (src == country_code::France) {
    dst = country_code_t::FR;
    return true;
  }
  else if (src == country_code::French_Polynesia) {
    dst = country_code_t::PF;
    return true;
  }
  else if (src == country_code::Gabon) {
    dst = country_code_t::GA;
    return true;
  }
  else if (src == country_code::Gambia) {
    dst = country_code_t::GM;
    return true;
  }
  else if (src == country_code::Georgia) {
    dst = country_code_t::GE;
    return true;
  }
  else if (src == country_code::Germany) {
    dst = country_code_t::DE;
    return true;
  }
  else if (src == country_code::Ghana) {
    dst = country_code_t::GH;
    return true;
  }
  else if (src == country_code::Gibraltar) {
    dst = country_code_t::GI;
    return true;
  }
  else if (src == country_code::Greece) {
    dst = country_code_t::GR;
    return true;
  }
  else if (src == country_code::Greenland) {
    dst = country_code_t::GL;
    return true;
  }
  else if (src == country_code::Grenada) {
    dst = country_code_t::GD;
    return true;
  }
  else if (src == country_code::Guam) {
    dst = country_code_t::GU;
    return true;
  }
  else if (src == country_code::Guatemala) {
    dst = country_code_t::GT;
    return true;
  }
  else if (src == country_code::Guernsey) {
    dst = country_code_t::GG;
    return true;
  }
  else if (src == country_code::Guinea) {
    dst = country_code_t::GN;
    return true;
  }
  else if (src == country_code::Guinea_Bissau) {
    dst = country_code_t::GW;
    return true;
  }
  else if (src == country_code::Guyana) {
    dst = country_code_t::GY;
    return true;
  }
  else if (src == country_code::Haiti) {
    dst = country_code_t::HT;
    return true;
  }
  else if (src == country_code::Honduras) {
    dst = country_code_t::HN;
    return true;
  }
  else if (src == country_code::Hong_Kong) {
    dst = country_code_t::HK;
    return true;
  }
  else if (src == country_code::Hungary) {
    dst = country_code_t::HU;
    return true;
  }
  else if (src == country_code::Iceland) {
    dst = country_code_t::IS;
    return true;
  }
  else if (src == country_code::India) {
    dst = country_code_t::IN;
    return true;
  }
  else if (src == country_code::Indonesia) {
    dst = country_code_t::ID;
    return true;
  }
  else if (src == country_code::Iran) {
    dst = country_code_t::IR;
    return true;
  }
  else if (src == country_code::Iraq) {
    dst = country_code_t::IQ;
    return true;
  }
  else if (src == country_code::Ireland) {
    dst = country_code_t::IE;
    return true;
  }
  else if (src == country_code::Isle_of_Man) {
    dst = country_code_t::IM;
    return true;
  }
  else if (src == country_code::Israel) {
    dst = country_code_t::IL;
    return true;
  }
  else if (src == country_code::Italy) {
    dst = country_code_t::IT;
    return true;
  }
  else if (src == country_code::Ivory_Coast) {
    dst = country_code_t::CI;
    return true;
  }
  else if (src == country_code::Jamaica) {
    dst = country_code_t::JM;
    return true;
  }
  else if (src == country_code::Japan) {
    dst = country_code_t::JP;
    return true;
  }
  else if (src == country_code::Jersey) {
    dst = country_code_t::JE;
    return true;
  }
  else if (src == country_code::Jordan) {
    dst = country_code_t::JO;
    return true;
  }
  else if (src == country_code::Kazakhsta) {
    dst = country_code_t::KZ;
    return true;
  }
  else if (src == country_code::Kenya) {
    dst = country_code_t::KE;
    return true;
  }
  else if (src == country_code::Kiribati) {
    dst = country_code_t::KI;
    return true;
  }
  else if (src == country_code::Kosovo) {
    dst = country_code_t::XK;
    return true;
  }
  else if (src == country_code::Kuwait) {
    dst = country_code_t::KW;
    return true;
  }
  else if (src == country_code::Kyrgyzstan) {
    dst = country_code_t::KG;
    return true;
  }
  else if (src == country_code::Laos) {
    dst = country_code_t::LA;
    return true;
  }
  else if (src == country_code::Latvia) {
    dst = country_code_t::LV;
    return true;
  }
  else if (src == country_code::Lebanon) {
    dst = country_code_t::LB;
    return true;
  }
  else if (src == country_code::Lesotho) {
    dst = country_code_t::LS;
    return true;
  }
  else if (src == country_code::Liberia) {
    dst = country_code_t::LR;
    return true;
  }
  else if (src == country_code::Libya) {
    dst = country_code_t::LY;
    return true;
  }
  else if (src == country_code::Liechtenstein) {
    dst = country_code_t::LI;
    return true;
  }
  else if (src == country_code::Lithuania) {
    dst = country_code_t::LT;
    return true;
  }
  else if (src == country_code::Luxembourg) {
    dst = country_code_t::LU;
    return true;
  }
  else if (src == country_code::Macau) {
    dst = country_code_t::MO;
    return true;
  }
  else if (src == country_code::Macedonia) {
    dst = country_code_t::MK;
    return true;
  }
  else if (src == country_code::Madagascar) {
    dst = country_code_t::MG;
    return true;
  }
  else if (src == country_code::Malawi) {
    dst = country_code_t::MW;
    return true;
  }
  else if (src == country_code::Malaysia) {
    dst = country_code_t::MY;
    return true;
  }
  else if (src == country_code::Maldives) {
    dst = country_code_t::MV;
    return true;
  }
  else if (src == country_code::Mali) {
    dst = country_code_t::ML;
    return true;
  }
  else if (src == country_code::Malta) {
    dst = country_code_t::MT;
    return true;
  }
  else if (src == country_code::Marshall_Islands) {
    dst = country_code_t::MH;
    return true;
  }
  else if (src == country_code::Mauritania) {
    dst = country_code_t::MR;
    return true;
  }
  else if (src == country_code::Mauritius) {
    dst = country_code_t::MU;
    return true;
  }
  else if (src == country_code::Mayotte) {
    dst = country_code_t::YT;
    return true;
  }
  else if (src == country_code::Mexico) {
    dst = country_code_t::MX;
    return true;
  }
  else if (src == country_code::Micronesia) {
    dst = country_code_t::FM;
    return true;
  }
  else if (src == country_code::Moldova) {
    dst = country_code_t::MD;
    return true;
  }
  else if (src == country_code::Monaco) {
    dst = country_code_t::MC;
    return true;
  }
  else if (src == country_code::Mongolia) {
    dst = country_code_t::MN;
    return true;
  }
  else if (src == country_code::Montenegro) {
    dst = country_code_t::ME;
    return true;
  }
  else if (src == country_code::Montserrat) {
    dst = country_code_t::MS;
    return true;
  }
  else if (src == country_code::Morocco) {
    dst = country_code_t::MA;
    return true;
  }
  else if (src == country_code::Mozambique) {
    dst = country_code_t::MZ;
    return true;
  }
  else if (src == country_code::Myanmar) {
    dst = country_code_t::MM;
    return true;
  }
  else if (src == country_code::Namibia) {
    dst = country_code_t::NA;
    return true;
  }
  else if (src == country_code::Nauru) {
    dst = country_code_t::NR;
    return true;
  }
  else if (src == country_code::Nepal) {
    dst = country_code_t::NP;
    return true;
  }
  else if (src == country_code::Netherlands) {
    dst = country_code_t::NL;
    return true;
  }
  else if (src == country_code::Netherlands_Antilles) {
    dst = country_code_t::AN;
    return true;
  }
  else if (src == country_code::New_Caledonia) {
    dst = country_code_t::NC;
    return true;
  }
  else if (src == country_code::New_Zealand) {
    dst = country_code_t::NZ;
    return true;
  }
  else if (src == country_code::Nicaragua) {
    dst = country_code_t::NI;
    return true;
  }
  else if (src == country_code::Niger) {
    dst = country_code_t::NE;
    return true;
  }
  else if (src == country_code::Nigeria) {
    dst = country_code_t::NG;
    return true;
  }
  else if (src == country_code::Niue) {
    dst = country_code_t::NU;
    return true;
  }
  else if (src == country_code::North_Korea) {
    dst = country_code_t::KP;
    return true;
  }
  else if (src == country_code::Northern_Mariana_Islands) {
    dst = country_code_t::MP;
    return true;
  }
  else if (src == country_code::Norway) {
    dst = country_code_t::NO;
    return true;
  }
  else if (src == country_code::Oman) {
    dst = country_code_t::OM;
    return true;
  }
  else if (src == country_code::Pakistan) {
    dst = country_code_t::PK;
    return true;
  }
  else if (src == country_code::Palau) {
    dst = country_code_t::PW;
    return true;
  }
  else if (src == country_code::Palestine) {
    dst = country_code_t::PS;
    return true;
  }
  else if (src == country_code::Panama) {
    dst = country_code_t::PA;
    return true;
  }
  else if (src == country_code::Papua_New_Guinea) {
    dst = country_code_t::PG;
    return true;
  }
  else if (src == country_code::Paraguay) {
    dst = country_code_t::PY;
    return true;
  }
  else if (src == country_code::Peru) {
    dst = country_code_t::PE;
    return true;
  }
  else if (src == country_code::Philippines) {
    dst = country_code_t::PH;
    return true;
  }
  else if (src == country_code::Pitcairn) {
    dst = country_code_t::PN;
    return true;
  }
  else if (src == country_code::Poland) {
    dst = country_code_t::PL;
    return true;
  }
  else if (src == country_code::Portugal) {
    dst = country_code_t::PT;
    return true;
  }
  else if (src == country_code::Puerto_Rico) {
    dst = country_code_t::PR;
    return true;
  }
  else if (src == country_code::Qatar) {
    dst = country_code_t::QA;
    return true;
  }
  else if (src == country_code::Republic_of_the_Congo) {
    dst = country_code_t::CG;
    return true;
  }
  else if (src == country_code::Reunion) {
    dst = country_code_t::RE;
    return true;
  }
  else if (src == country_code::Romania) {
    dst = country_code_t::RO;
    return true;
  }
  else if (src == country_code::Russi) {
    dst = country_code_t::RU;
    return true;
  }
  else if (src == country_code::Rwanda) {
    dst = country_code_t::RW;
    return true;
  }
  else if (src == country_code::Saint_Barthelemy) {
    dst = country_code_t::BL;
    return true;
  }
  else if (src == country_code::Saint_Helena) {
    dst = country_code_t::SH;
    return true;
  }
  else if (src == country_code::Saint_Kitts_and_Nevis) {
    dst = country_code_t::KN;
    return true;
  }
  else if (src == country_code::Saint_Lucia) {
    dst = country_code_t::LC;
    return true;
  }
  else if (src == country_code::Saint_Martin) {
    dst = country_code_t::MF;
    return true;
  }
  else if (src == country_code::Saint_Pierre_and_Miquelon) {
    dst = country_code_t::PM;
    return true;
  }
  else if (src == country_code::Saint_Vincent_and_the_Grenadines) {
    dst = country_code_t::VC;
    return true;
  }
  else if (src == country_code::Samoa) {
    dst = country_code_t::WS;
    return true;
  }
  else if (src == country_code::San_Marino) {
    dst = country_code_t::SM;
    return true;
  }
  else if (src == country_code::Sao_Tome_and_Principe) {
    dst = country_code_t::ST;
    return true;
  }
  else if (src == country_code::Saudi_Arabia) {
    dst = country_code_t::SA;
    return true;
  }
  else if (src == country_code::Senegal) {
    dst = country_code_t::SN;
    return true;
  }
  else if (src == country_code::Serbia) {
    dst = country_code_t::RS;
    return true;
  }
  else if (src == country_code::Seychelles) {
    dst = country_code_t::SC;
    return true;
  }
  else if (src == country_code::Sierra_Leone) {
    dst = country_code_t::SL;
    return true;
  }
  else if (src == country_code::Singapore) {
    dst = country_code_t::SG;
    return true;
  }
  else if (src == country_code::Sint_Maarten) {
    dst = country_code_t::SX;
    return true;
  }
  else if (src == country_code::Slovakia) {
    dst = country_code_t::SK;
    return true;
  }
  else if (src == country_code::Slovenia) {
    dst = country_code_t::SI;
    return true;
  }
  else if (src == country_code::Solomon_Islands) {
    dst = country_code_t::SB;
    return true;
  }
  else if (src == country_code::Somalia) {
    dst = country_code_t::SO;
    return true;
  }
  else if (src == country_code::South_Africa) {
    dst = country_code_t::ZA;
    return true;
  }
  else if (src == country_code::South_Korea) {
    dst = country_code_t::KR;
    return true;
  }
  else if (src == country_code::South_Sudan) {
    dst = country_code_t::SS;
    return true;
  }
  else if (src == country_code::Spain) {
    dst = country_code_t::ES;
    return true;
  }
  else if (src == country_code::Sri_Lanka) {
    dst = country_code_t::LK;
    return true;
  }
  else if (src == country_code::Sudan) {
    dst = country_code_t::SD;
    return true;
  }
  else if (src == country_code::Suriname) {
    dst = country_code_t::SR;
    return true;
  }
  else if (src == country_code::Svalbard_and_Jan_Mayen) {
    dst = country_code_t::SJ;
    return true;
  }
  else if (src == country_code::Swaziland) {
    dst = country_code_t::SZ;
    return true;
  }
  else if (src == country_code::Sweden) {
    dst = country_code_t::SE;
    return true;
  }
  else if (src == country_code::Switzerland) {
    dst = country_code_t::CH;
    return true;
  }
  else if (src == country_code::Syria) {
    dst = country_code_t::SY;
    return true;
  }
  else if (src == country_code::Taiwan) {
    dst = country_code_t::TW;
    return true;
  }
  else if (src == country_code::Tajikistan) {
    dst = country_code_t::TJ;
    return true;
  }
  else if (src == country_code::Tanzania) {
    dst = country_code_t::TZ;
    return true;
  }
  else if (src == country_code::Thailand) {
    dst = country_code_t::TH;
    return true;
  }
  else if (src == country_code::Togo) {
    dst = country_code_t::TG;
    return true;
  }
  else if (src == country_code::Tokelau) {
    dst = country_code_t::TK;
    return true;
  }
  else if (src == country_code::Tonga) {
    dst = country_code_t::TO;
    return true;
  }
  else if (src == country_code::Trinidad_and_Tobago) {
    dst = country_code_t::TT;
    return true;
  }
  else if (src == country_code::Tunisia) {
    dst = country_code_t::TN;
    return true;
  }
  else if (src == country_code::Turkey) {
    dst = country_code_t::TR;
    return true;
  }
  else if (src == country_code::Turkmenistan) {
    dst = country_code_t::TM;
    return true;
  }
  else if (src == country_code::Turks_and_Caicos_Islands) {
    dst = country_code_t::TC;
    return true;
  }
  else if (src == country_code::Tuvalu) {
    dst = country_code_t::TV;
    return true;
  }
  else if (src == country_code::US_Virgin_Islands) {
    dst = country_code_t::VI;
    return true;
  }
  else if (src == country_code::Uganda) {
    dst = country_code_t::UG;
    return true;
  }
  else if (src == country_code::Ukraine) {
    dst = country_code_t::UA;
    return true;
  }
  else if (src == country_code::United_Arab_Emirates) {
    dst = country_code_t::AE;
    return true;
  }
  else if (src == country_code::United_Kingdom) {
    dst = country_code_t::GB;
    return true;
  }
  else if (src == country_code::United_State) {
    dst = country_code_t::US;
    return true;
  }
  else if (src == country_code::Uruguay) {
    dst = country_code_t::UY;
    return true;
  }
  else if (src == country_code::Uzbekistan) {
    dst = country_code_t::UZ;
    return true;
  }
  else if (src == country_code::Vanuatu) {
    dst = country_code_t::VU;
    return true;
  }
  else if (src == country_code::Vatican) {
    dst = country_code_t::VA;
    return true;
  }
  else if (src == country_code::Venezuela) {
    dst = country_code_t::VE;
    return true;
  }
  else if (src == country_code::Vietnam) {
    dst = country_code_t::VN;
    return true;
  }
  else if (src == country_code::Wallis_and_Futuna) {
    dst = country_code_t::WF;
    return true;
  }
  else if (src == country_code::Western_Sahara) {
    dst = country_code_t::EH;
    return true;
  }
  else if (src == country_code::Yemen) {
    dst = country_code_t::YE;
    return true;
  }
  else if (src == country_code::Zambia) {
    dst = country_code_t::ZM;
    return true;
  }
  else if (src == country_code::Zimbabwe) {
    dst = country_code_t::ZW;
    return true;
  }
  else {
    cerr << "Invalid country code: " << src << endl;
    return false;
  }

  return true;
}

bool WpaSupplicantParser::parseKeyManagement(const string& src, key_mgmt_t& dst)
{
  if (src == key_mgmt::WPA_PSK) {
    dst = key_mgmt_t::WPA_PSK;
  }
  else if (src == key_mgmt::WPA_EAP) {
    dst = key_mgmt_t::WPA_EAP;
  }
  else {
    cerr << "Invalid key management method: " << src << endl;
    return false;
  }

  return true;
}

const char* WpaSupplicantParser::countryCodeToString(country_code_t cc)
{
  switch (cc) {
    case country_code_t::AF:
      return country_code::Afghanistan;
    case country_code_t::AL:
      return country_code::Albania;
    case country_code_t::DZ:
      return country_code::Algeria;
    case country_code_t::AS:
      return country_code::American_Samoa;
    case country_code_t::AD:
      return country_code::Andorra;
    case country_code_t::AO:
      return country_code::Angola;
    case country_code_t::AI:
      return country_code::Anguilla;
    case country_code_t::AQ:
      return country_code::Antarctica;
    case country_code_t::AG:
      return country_code::Antigua_and_Barbuda;
    case country_code_t::AR:
      return country_code::Argentina;
    case country_code_t::AM:
      return country_code::Armenia;
    case country_code_t::AW:
      return country_code::Aruba;
    case country_code_t::AU:
      return country_code::Australia;
    case country_code_t::AT:
      return country_code::Austria;
    case country_code_t::AZ:
      return country_code::Azerbaijan;
    case country_code_t::BS:
      return country_code::Bahamas;
    case country_code_t::BH:
      return country_code::Bahrain;
    case country_code_t::BD:
      return country_code::Bangladesh;
    case country_code_t::BB:
      return country_code::Barbados;
    case country_code_t::BY:
      return country_code::Belarus;
    case country_code_t::BE:
      return country_code::Belgium;
    case country_code_t::BZ:
      return country_code::Belize;
    case country_code_t::BJ:
      return country_code::Benin;
    case country_code_t::BM:
      return country_code::Bermuda;
    case country_code_t::BT:
      return country_code::Bhutan;
    case country_code_t::BO:
      return country_code::Bolivia;
    case country_code_t::BA:
      return country_code::Bosnia_and_Herzegovina;
    case country_code_t::BW:
      return country_code::Botswana;
    case country_code_t::BR:
      return country_code::Brazil;
    case country_code_t::IO:
      return country_code::British_Indian_OceanTerritory;
    case country_code_t::VG:
      return country_code::British_Virgin_Islands;
    case country_code_t::BN:
      return country_code::Brunei;
    case country_code_t::BG:
      return country_code::Bulgaria;
    case country_code_t::BF:
      return country_code::Burkina_Faso;
    case country_code_t::BI:
      return country_code::Burundi;
    case country_code_t::KH:
      return country_code::Cambodia;
    case country_code_t::CM:
      return country_code::Cameroon;
    case country_code_t::CA:
      return country_code::Canad;
    case country_code_t::CV:
      return country_code::Cape_Verde;
    case country_code_t::KY:
      return country_code::Cayman_Islands;
    case country_code_t::CF:
      return country_code::Central_African_Republic;
    case country_code_t::TD:
      return country_code::Chad;
    case country_code_t::CL:
      return country_code::Chile;
    case country_code_t::CN:
      return country_code::China;
    case country_code_t::CX:
      return country_code::Christmas_Island;
    case country_code_t::CC:
      return country_code::Cocos_Islands;
    case country_code_t::CO:
      return country_code::Colombia;
    case country_code_t::KM:
      return country_code::Comoros;
    case country_code_t::CK:
      return country_code::Cook_Islands;
    case country_code_t::CR:
      return country_code::Costa_Rica;
    case country_code_t::HR:
      return country_code::Croatia;
    case country_code_t::CU:
      return country_code::Cuba;
    case country_code_t::CW:
      return country_code::Curacao;
    case country_code_t::CY:
      return country_code::Cyprus;
    case country_code_t::CZ:
      return country_code::Czech_Republic;
    case country_code_t::CD:
      return country_code::Democratic_Republic_of_the_Congo;
    case country_code_t::DK:
      return country_code::Denmark;
    case country_code_t::DJ:
      return country_code::Djibouti;
    case country_code_t::DM:
      return country_code::Dominica;
    case country_code_t::DO:
      return country_code::Dominican_Republic;
    case country_code_t::TL:
      return country_code::East_Timor;
    case country_code_t::EC:
      return country_code::Ecuador;
    case country_code_t::EG:
      return country_code::Egypt;
    case country_code_t::SV:
      return country_code::El_Salvador;
    case country_code_t::GQ:
      return country_code::Equatorial_Guinea;
    case country_code_t::ER:
      return country_code::Eritrea;
    case country_code_t::EE:
      return country_code::Estonia;
    case country_code_t::ET:
      return country_code::Ethiopia;
    case country_code_t::FK:
      return country_code::Falkland_Islands;
    case country_code_t::FO:
      return country_code::Faroe_Islands;
    case country_code_t::FJ:
      return country_code::Fiji;
    case country_code_t::FI:
      return country_code::Finland;
    case country_code_t::FR:
      return country_code::France;
    case country_code_t::PF:
      return country_code::French_Polynesia;
    case country_code_t::GA:
      return country_code::Gabon;
    case country_code_t::GM:
      return country_code::Gambia;
    case country_code_t::GE:
      return country_code::Georgia;
    case country_code_t::DE:
      return country_code::Germany;
    case country_code_t::GH:
      return country_code::Ghana;
    case country_code_t::GI:
      return country_code::Gibraltar;
    case country_code_t::GR:
      return country_code::Greece;
    case country_code_t::GL:
      return country_code::Greenland;
    case country_code_t::GD:
      return country_code::Grenada;
    case country_code_t::GU:
      return country_code::Guam;
    case country_code_t::GT:
      return country_code::Guatemala;
    case country_code_t::GG:
      return country_code::Guernsey;
    case country_code_t::GN:
      return country_code::Guinea;
    case country_code_t::GW:
      return country_code::Guinea_Bissau;
    case country_code_t::GY:
      return country_code::Guyana;
    case country_code_t::HT:
      return country_code::Haiti;
    case country_code_t::HN:
      return country_code::Honduras;
    case country_code_t::HK:
      return country_code::Hong_Kong;
    case country_code_t::HU:
      return country_code::Hungary;
    case country_code_t::IS:
      return country_code::Iceland;
    case country_code_t::IN:
      return country_code::India;
    case country_code_t::ID:
      return country_code::Indonesia;
    case country_code_t::IR:
      return country_code::Iran;
    case country_code_t::IQ:
      return country_code::Iraq;
    case country_code_t::IE:
      return country_code::Ireland;
    case country_code_t::IM:
      return country_code::Isle_of_Man;
    case country_code_t::IL:
      return country_code::Israel;
    case country_code_t::IT:
      return country_code::Italy;
    case country_code_t::CI:
      return country_code::Ivory_Coast;
    case country_code_t::JM:
      return country_code::Jamaica;
    case country_code_t::JP:
      return country_code::Japan;
    case country_code_t::JE:
      return country_code::Jersey;
    case country_code_t::JO:
      return country_code::Jordan;
    case country_code_t::KZ:
      return country_code::Kazakhsta;
    case country_code_t::KE:
      return country_code::Kenya;
    case country_code_t::KI:
      return country_code::Kiribati;
    case country_code_t::XK:
      return country_code::Kosovo;
    case country_code_t::KW:
      return country_code::Kuwait;
    case country_code_t::KG:
      return country_code::Kyrgyzstan;
    case country_code_t::LA:
      return country_code::Laos;
    case country_code_t::LV:
      return country_code::Latvia;
    case country_code_t::LB:
      return country_code::Lebanon;
    case country_code_t::LS:
      return country_code::Lesotho;
    case country_code_t::LR:
      return country_code::Liberia;
    case country_code_t::LY:
      return country_code::Libya;
    case country_code_t::LI:
      return country_code::Liechtenstein;
    case country_code_t::LT:
      return country_code::Lithuania;
    case country_code_t::LU:
      return country_code::Luxembourg;
    case country_code_t::MO:
      return country_code::Macau;
    case country_code_t::MK:
      return country_code::Macedonia;
    case country_code_t::MG:
      return country_code::Madagascar;
    case country_code_t::MW:
      return country_code::Malawi;
    case country_code_t::MY:
      return country_code::Malaysia;
    case country_code_t::MV:
      return country_code::Maldives;
    case country_code_t::ML:
      return country_code::Mali;
    case country_code_t::MT:
      return country_code::Malta;
    case country_code_t::MH:
      return country_code::Marshall_Islands;
    case country_code_t::MR:
      return country_code::Mauritania;
    case country_code_t::MU:
      return country_code::Mauritius;
    case country_code_t::YT:
      return country_code::Mayotte;
    case country_code_t::MX:
      return country_code::Mexico;
    case country_code_t::FM:
      return country_code::Micronesia;
    case country_code_t::MD:
      return country_code::Moldova;
    case country_code_t::MC:
      return country_code::Monaco;
    case country_code_t::MN:
      return country_code::Mongolia;
    case country_code_t::ME:
      return country_code::Montenegro;
    case country_code_t::MS:
      return country_code::Montserrat;
    case country_code_t::MA:
      return country_code::Morocco;
    case country_code_t::MZ:
      return country_code::Mozambique;
    case country_code_t::MM:
      return country_code::Myanmar;
    case country_code_t::NA:
      return country_code::Namibia;
    case country_code_t::NR:
      return country_code::Nauru;
    case country_code_t::NP:
      return country_code::Nepal;
    case country_code_t::NL:
      return country_code::Netherlands;
    case country_code_t::AN:
      return country_code::Netherlands_Antilles;
    case country_code_t::NC:
      return country_code::New_Caledonia;
    case country_code_t::NZ:
      return country_code::New_Zealand;
    case country_code_t::NI:
      return country_code::Nicaragua;
    case country_code_t::NE:
      return country_code::Niger;
    case country_code_t::NG:
      return country_code::Nigeria;
    case country_code_t::NU:
      return country_code::Niue;
    case country_code_t::KP:
      return country_code::North_Korea;
    case country_code_t::MP:
      return country_code::Northern_Mariana_Islands;
    case country_code_t::NO:
      return country_code::Norway;
    case country_code_t::OM:
      return country_code::Oman;
    case country_code_t::PK:
      return country_code::Pakistan;
    case country_code_t::PW:
      return country_code::Palau;
    case country_code_t::PS:
      return country_code::Palestine;
    case country_code_t::PA:
      return country_code::Panama;
    case country_code_t::PG:
      return country_code::Papua_New_Guinea;
    case country_code_t::PY:
      return country_code::Paraguay;
    case country_code_t::PE:
      return country_code::Peru;
    case country_code_t::PH:
      return country_code::Philippines;
    case country_code_t::PN:
      return country_code::Pitcairn;
    case country_code_t::PL:
      return country_code::Poland;
    case country_code_t::PT:
      return country_code::Portugal;
    case country_code_t::PR:
      return country_code::Puerto_Rico;
    case country_code_t::QA:
      return country_code::Qatar;
    case country_code_t::CG:
      return country_code::Republic_of_the_Congo;
    case country_code_t::RE:
      return country_code::Reunion;
    case country_code_t::RO:
      return country_code::Romania;
    case country_code_t::RU:
      return country_code::Russi;
    case country_code_t::RW:
      return country_code::Rwanda;
    case country_code_t::BL:
      return country_code::Saint_Barthelemy;
    case country_code_t::SH:
      return country_code::Saint_Helena;
    case country_code_t::KN:
      return country_code::Saint_Kitts_and_Nevis;
    case country_code_t::LC:
      return country_code::Saint_Lucia;
    case country_code_t::MF:
      return country_code::Saint_Martin;
    case country_code_t::PM:
      return country_code::Saint_Pierre_and_Miquelon;
    case country_code_t::VC:
      return country_code::Saint_Vincent_and_the_Grenadines;
    case country_code_t::WS:
      return country_code::Samoa;
    case country_code_t::SM:
      return country_code::San_Marino;
    case country_code_t::ST:
      return country_code::Sao_Tome_and_Principe;
    case country_code_t::SA:
      return country_code::Saudi_Arabia;
    case country_code_t::SN:
      return country_code::Senegal;
    case country_code_t::RS:
      return country_code::Serbia;
    case country_code_t::SC:
      return country_code::Seychelles;
    case country_code_t::SL:
      return country_code::Sierra_Leone;
    case country_code_t::SG:
      return country_code::Singapore;
    case country_code_t::SX:
      return country_code::Sint_Maarten;
    case country_code_t::SK:
      return country_code::Slovakia;
    case country_code_t::SI:
      return country_code::Slovenia;
    case country_code_t::SB:
      return country_code::Solomon_Islands;
    case country_code_t::SO:
      return country_code::Somalia;
    case country_code_t::ZA:
      return country_code::South_Africa;
    case country_code_t::KR:
      return country_code::South_Korea;
    case country_code_t::SS:
      return country_code::South_Sudan;
    case country_code_t::ES:
      return country_code::Spain;
    case country_code_t::LK:
      return country_code::Sri_Lanka;
    case country_code_t::SD:
      return country_code::Sudan;
    case country_code_t::SR:
      return country_code::Suriname;
    case country_code_t::SJ:
      return country_code::Svalbard_and_Jan_Mayen;
    case country_code_t::SZ:
      return country_code::Swaziland;
    case country_code_t::SE:
      return country_code::Sweden;
    case country_code_t::CH:
      return country_code::Switzerland;
    case country_code_t::SY:
      return country_code::Syria;
    case country_code_t::TW:
      return country_code::Taiwan;
    case country_code_t::TJ:
      return country_code::Tajikistan;
    case country_code_t::TZ:
      return country_code::Tanzania;
    case country_code_t::TH:
      return country_code::Thailand;
    case country_code_t::TG:
      return country_code::Togo;
    case country_code_t::TK:
      return country_code::Tokelau;
    case country_code_t::TO:
      return country_code::Tonga;
    case country_code_t::TT:
      return country_code::Trinidad_and_Tobago;
    case country_code_t::TN:
      return country_code::Tunisia;
    case country_code_t::TR:
      return country_code::Turkey;
    case country_code_t::TM:
      return country_code::Turkmenistan;
    case country_code_t::TC:
      return country_code::Turks_and_Caicos_Islands;
    case country_code_t::TV:
      return country_code::Tuvalu;
    case country_code_t::VI:
      return country_code::US_Virgin_Islands;
    case country_code_t::UG:
      return country_code::Uganda;
    case country_code_t::UA:
      return country_code::Ukraine;
    case country_code_t::AE:
      return country_code::United_Arab_Emirates;
    case country_code_t::GB:
      return country_code::United_Kingdom;
    case country_code_t::US:
      return country_code::United_State;
    case country_code_t::UY:
      return country_code::Uruguay;
    case country_code_t::UZ:
      return country_code::Uzbekistan;
    case country_code_t::VU:
      return country_code::Vanuatu;
    case country_code_t::VA:
      return country_code::Vatican;
    case country_code_t::VE:
      return country_code::Venezuela;
    case country_code_t::VN:
      return country_code::Vietnam;
    case country_code_t::WF:
      return country_code::Wallis_and_Futuna;
    case country_code_t::EH:
      return country_code::Western_Sahara;
    case country_code_t::YE:
      return country_code::Yemen;
    case country_code_t::ZM:
      return country_code::Zambia;
    case country_code_t::ZW:
      return country_code::Zimbabwe;
    default:
      throw;
  }
}

const char* WpaSupplicantParser::keyManagementToString(key_mgmt_t key_mgmt)
{
  switch (key_mgmt) {
    case key_mgmt_t::WPA_PSK:
      return key_mgmt::WPA_PSK;
    case key_mgmt_t::WPA_EAP:
      return key_mgmt::WPA_EAP;
    default:
      throw;
  }
}
}  // namespace wpa
