#include "tobas_wpa_supplicant_parser/parser.hpp"

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

bool WpaSupplicantParser::parseCountryCode(const string& src, CountryCode& dst)
{
  if (src == country_code::Afghanistan) {
    dst = CountryCode::AF;
    return true;
  }
  else if (src == country_code::Albania) {
    dst = CountryCode::AL;
    return true;
  }
  else if (src == country_code::Algeria) {
    dst = CountryCode::DZ;
    return true;
  }
  else if (src == country_code::American_Samoa) {
    dst = CountryCode::AS;
    return true;
  }
  else if (src == country_code::Andorra) {
    dst = CountryCode::AD;
    return true;
  }
  else if (src == country_code::Angola) {
    dst = CountryCode::AO;
    return true;
  }
  else if (src == country_code::Anguilla) {
    dst = CountryCode::AI;
    return true;
  }
  else if (src == country_code::Antarctica) {
    dst = CountryCode::AQ;
    return true;
  }
  else if (src == country_code::Antigua_and_Barbuda) {
    dst = CountryCode::AG;
    return true;
  }
  else if (src == country_code::Argentina) {
    dst = CountryCode::AR;
    return true;
  }
  else if (src == country_code::Armenia) {
    dst = CountryCode::AM;
    return true;
  }
  else if (src == country_code::Aruba) {
    dst = CountryCode::AW;
    return true;
  }
  else if (src == country_code::Australia) {
    dst = CountryCode::AU;
    return true;
  }
  else if (src == country_code::Austria) {
    dst = CountryCode::AT;
    return true;
  }
  else if (src == country_code::Azerbaijan) {
    dst = CountryCode::AZ;
    return true;
  }
  else if (src == country_code::Bahamas) {
    dst = CountryCode::BS;
    return true;
  }
  else if (src == country_code::Bahrain) {
    dst = CountryCode::BH;
    return true;
  }
  else if (src == country_code::Bangladesh) {
    dst = CountryCode::BD;
    return true;
  }
  else if (src == country_code::Barbados) {
    dst = CountryCode::BB;
    return true;
  }
  else if (src == country_code::Belarus) {
    dst = CountryCode::BY;
    return true;
  }
  else if (src == country_code::Belgium) {
    dst = CountryCode::BE;
    return true;
  }
  else if (src == country_code::Belize) {
    dst = CountryCode::BZ;
    return true;
  }
  else if (src == country_code::Benin) {
    dst = CountryCode::BJ;
    return true;
  }
  else if (src == country_code::Bermuda) {
    dst = CountryCode::BM;
    return true;
  }
  else if (src == country_code::Bhutan) {
    dst = CountryCode::BT;
    return true;
  }
  else if (src == country_code::Bolivia) {
    dst = CountryCode::BO;
    return true;
  }
  else if (src == country_code::Bosnia_and_Herzegovina) {
    dst = CountryCode::BA;
    return true;
  }
  else if (src == country_code::Botswana) {
    dst = CountryCode::BW;
    return true;
  }
  else if (src == country_code::Brazil) {
    dst = CountryCode::BR;
    return true;
  }
  else if (src == country_code::British_Indian_OceanTerritory) {
    dst = CountryCode::IO;
    return true;
  }
  else if (src == country_code::British_Virgin_Islands) {
    dst = CountryCode::VG;
    return true;
  }
  else if (src == country_code::Brunei) {
    dst = CountryCode::BN;
    return true;
  }
  else if (src == country_code::Bulgaria) {
    dst = CountryCode::BG;
    return true;
  }
  else if (src == country_code::Burkina_Faso) {
    dst = CountryCode::BF;
    return true;
  }
  else if (src == country_code::Burundi) {
    dst = CountryCode::BI;
    return true;
  }
  else if (src == country_code::Cambodia) {
    dst = CountryCode::KH;
    return true;
  }
  else if (src == country_code::Cameroon) {
    dst = CountryCode::CM;
    return true;
  }
  else if (src == country_code::Canad) {
    dst = CountryCode::CA;
    return true;
  }
  else if (src == country_code::Cape_Verde) {
    dst = CountryCode::CV;
    return true;
  }
  else if (src == country_code::Cayman_Islands) {
    dst = CountryCode::KY;
    return true;
  }
  else if (src == country_code::Central_African_Republic) {
    dst = CountryCode::CF;
    return true;
  }
  else if (src == country_code::Chad) {
    dst = CountryCode::TD;
    return true;
  }
  else if (src == country_code::Chile) {
    dst = CountryCode::CL;
    return true;
  }
  else if (src == country_code::China) {
    dst = CountryCode::CN;
    return true;
  }
  else if (src == country_code::Christmas_Island) {
    dst = CountryCode::CX;
    return true;
  }
  else if (src == country_code::Cocos_Islands) {
    dst = CountryCode::CC;
    return true;
  }
  else if (src == country_code::Colombia) {
    dst = CountryCode::CO;
    return true;
  }
  else if (src == country_code::Comoros) {
    dst = CountryCode::KM;
    return true;
  }
  else if (src == country_code::Cook_Islands) {
    dst = CountryCode::CK;
    return true;
  }
  else if (src == country_code::Costa_Rica) {
    dst = CountryCode::CR;
    return true;
  }
  else if (src == country_code::Croatia) {
    dst = CountryCode::HR;
    return true;
  }
  else if (src == country_code::Cuba) {
    dst = CountryCode::CU;
    return true;
  }
  else if (src == country_code::Curacao) {
    dst = CountryCode::CW;
    return true;
  }
  else if (src == country_code::Cyprus) {
    dst = CountryCode::CY;
    return true;
  }
  else if (src == country_code::Czech_Republic) {
    dst = CountryCode::CZ;
    return true;
  }
  else if (src == country_code::Democratic_Republic_of_the_Congo) {
    dst = CountryCode::CD;
    return true;
  }
  else if (src == country_code::Denmark) {
    dst = CountryCode::DK;
    return true;
  }
  else if (src == country_code::Djibouti) {
    dst = CountryCode::DJ;
    return true;
  }
  else if (src == country_code::Dominica) {
    dst = CountryCode::DM;
    return true;
  }
  else if (src == country_code::Dominican_Republic) {
    dst = CountryCode::DO;
    return true;
  }
  else if (src == country_code::East_Timor) {
    dst = CountryCode::TL;
    return true;
  }
  else if (src == country_code::Ecuador) {
    dst = CountryCode::EC;
    return true;
  }
  else if (src == country_code::Egypt) {
    dst = CountryCode::EG;
    return true;
  }
  else if (src == country_code::El_Salvador) {
    dst = CountryCode::SV;
    return true;
  }
  else if (src == country_code::Equatorial_Guinea) {
    dst = CountryCode::GQ;
    return true;
  }
  else if (src == country_code::Eritrea) {
    dst = CountryCode::ER;
    return true;
  }
  else if (src == country_code::Estonia) {
    dst = CountryCode::EE;
    return true;
  }
  else if (src == country_code::Ethiopia) {
    dst = CountryCode::ET;
    return true;
  }
  else if (src == country_code::Falkland_Islands) {
    dst = CountryCode::FK;
    return true;
  }
  else if (src == country_code::Faroe_Islands) {
    dst = CountryCode::FO;
    return true;
  }
  else if (src == country_code::Fiji) {
    dst = CountryCode::FJ;
    return true;
  }
  else if (src == country_code::Finland) {
    dst = CountryCode::FI;
    return true;
  }
  else if (src == country_code::France) {
    dst = CountryCode::FR;
    return true;
  }
  else if (src == country_code::French_Polynesia) {
    dst = CountryCode::PF;
    return true;
  }
  else if (src == country_code::Gabon) {
    dst = CountryCode::GA;
    return true;
  }
  else if (src == country_code::Gambia) {
    dst = CountryCode::GM;
    return true;
  }
  else if (src == country_code::Georgia) {
    dst = CountryCode::GE;
    return true;
  }
  else if (src == country_code::Germany) {
    dst = CountryCode::DE;
    return true;
  }
  else if (src == country_code::Ghana) {
    dst = CountryCode::GH;
    return true;
  }
  else if (src == country_code::Gibraltar) {
    dst = CountryCode::GI;
    return true;
  }
  else if (src == country_code::Greece) {
    dst = CountryCode::GR;
    return true;
  }
  else if (src == country_code::Greenland) {
    dst = CountryCode::GL;
    return true;
  }
  else if (src == country_code::Grenada) {
    dst = CountryCode::GD;
    return true;
  }
  else if (src == country_code::Guam) {
    dst = CountryCode::GU;
    return true;
  }
  else if (src == country_code::Guatemala) {
    dst = CountryCode::GT;
    return true;
  }
  else if (src == country_code::Guernsey) {
    dst = CountryCode::GG;
    return true;
  }
  else if (src == country_code::Guinea) {
    dst = CountryCode::GN;
    return true;
  }
  else if (src == country_code::Guinea_Bissau) {
    dst = CountryCode::GW;
    return true;
  }
  else if (src == country_code::Guyana) {
    dst = CountryCode::GY;
    return true;
  }
  else if (src == country_code::Haiti) {
    dst = CountryCode::HT;
    return true;
  }
  else if (src == country_code::Honduras) {
    dst = CountryCode::HN;
    return true;
  }
  else if (src == country_code::Hong_Kong) {
    dst = CountryCode::HK;
    return true;
  }
  else if (src == country_code::Hungary) {
    dst = CountryCode::HU;
    return true;
  }
  else if (src == country_code::Iceland) {
    dst = CountryCode::IS;
    return true;
  }
  else if (src == country_code::India) {
    dst = CountryCode::IN;
    return true;
  }
  else if (src == country_code::Indonesia) {
    dst = CountryCode::ID;
    return true;
  }
  else if (src == country_code::Iran) {
    dst = CountryCode::IR;
    return true;
  }
  else if (src == country_code::Iraq) {
    dst = CountryCode::IQ;
    return true;
  }
  else if (src == country_code::Ireland) {
    dst = CountryCode::IE;
    return true;
  }
  else if (src == country_code::Isle_of_Man) {
    dst = CountryCode::IM;
    return true;
  }
  else if (src == country_code::Israel) {
    dst = CountryCode::IL;
    return true;
  }
  else if (src == country_code::Italy) {
    dst = CountryCode::IT;
    return true;
  }
  else if (src == country_code::Ivory_Coast) {
    dst = CountryCode::CI;
    return true;
  }
  else if (src == country_code::Jamaica) {
    dst = CountryCode::JM;
    return true;
  }
  else if (src == country_code::Japan) {
    dst = CountryCode::JP;
    return true;
  }
  else if (src == country_code::Jersey) {
    dst = CountryCode::JE;
    return true;
  }
  else if (src == country_code::Jordan) {
    dst = CountryCode::JO;
    return true;
  }
  else if (src == country_code::Kazakhsta) {
    dst = CountryCode::KZ;
    return true;
  }
  else if (src == country_code::Kenya) {
    dst = CountryCode::KE;
    return true;
  }
  else if (src == country_code::Kiribati) {
    dst = CountryCode::KI;
    return true;
  }
  else if (src == country_code::Kosovo) {
    dst = CountryCode::XK;
    return true;
  }
  else if (src == country_code::Kuwait) {
    dst = CountryCode::KW;
    return true;
  }
  else if (src == country_code::Kyrgyzstan) {
    dst = CountryCode::KG;
    return true;
  }
  else if (src == country_code::Laos) {
    dst = CountryCode::LA;
    return true;
  }
  else if (src == country_code::Latvia) {
    dst = CountryCode::LV;
    return true;
  }
  else if (src == country_code::Lebanon) {
    dst = CountryCode::LB;
    return true;
  }
  else if (src == country_code::Lesotho) {
    dst = CountryCode::LS;
    return true;
  }
  else if (src == country_code::Liberia) {
    dst = CountryCode::LR;
    return true;
  }
  else if (src == country_code::Libya) {
    dst = CountryCode::LY;
    return true;
  }
  else if (src == country_code::Liechtenstein) {
    dst = CountryCode::LI;
    return true;
  }
  else if (src == country_code::Lithuania) {
    dst = CountryCode::LT;
    return true;
  }
  else if (src == country_code::Luxembourg) {
    dst = CountryCode::LU;
    return true;
  }
  else if (src == country_code::Macau) {
    dst = CountryCode::MO;
    return true;
  }
  else if (src == country_code::Macedonia) {
    dst = CountryCode::MK;
    return true;
  }
  else if (src == country_code::Madagascar) {
    dst = CountryCode::MG;
    return true;
  }
  else if (src == country_code::Malawi) {
    dst = CountryCode::MW;
    return true;
  }
  else if (src == country_code::Malaysia) {
    dst = CountryCode::MY;
    return true;
  }
  else if (src == country_code::Maldives) {
    dst = CountryCode::MV;
    return true;
  }
  else if (src == country_code::Mali) {
    dst = CountryCode::ML;
    return true;
  }
  else if (src == country_code::Malta) {
    dst = CountryCode::MT;
    return true;
  }
  else if (src == country_code::Marshall_Islands) {
    dst = CountryCode::MH;
    return true;
  }
  else if (src == country_code::Mauritania) {
    dst = CountryCode::MR;
    return true;
  }
  else if (src == country_code::Mauritius) {
    dst = CountryCode::MU;
    return true;
  }
  else if (src == country_code::Mayotte) {
    dst = CountryCode::YT;
    return true;
  }
  else if (src == country_code::Mexico) {
    dst = CountryCode::MX;
    return true;
  }
  else if (src == country_code::Micronesia) {
    dst = CountryCode::FM;
    return true;
  }
  else if (src == country_code::Moldova) {
    dst = CountryCode::MD;
    return true;
  }
  else if (src == country_code::Monaco) {
    dst = CountryCode::MC;
    return true;
  }
  else if (src == country_code::Mongolia) {
    dst = CountryCode::MN;
    return true;
  }
  else if (src == country_code::Montenegro) {
    dst = CountryCode::ME;
    return true;
  }
  else if (src == country_code::Montserrat) {
    dst = CountryCode::MS;
    return true;
  }
  else if (src == country_code::Morocco) {
    dst = CountryCode::MA;
    return true;
  }
  else if (src == country_code::Mozambique) {
    dst = CountryCode::MZ;
    return true;
  }
  else if (src == country_code::Myanmar) {
    dst = CountryCode::MM;
    return true;
  }
  else if (src == country_code::Namibia) {
    dst = CountryCode::NA;
    return true;
  }
  else if (src == country_code::Nauru) {
    dst = CountryCode::NR;
    return true;
  }
  else if (src == country_code::Nepal) {
    dst = CountryCode::NP;
    return true;
  }
  else if (src == country_code::Netherlands) {
    dst = CountryCode::NL;
    return true;
  }
  else if (src == country_code::Netherlands_Antilles) {
    dst = CountryCode::AN;
    return true;
  }
  else if (src == country_code::New_Caledonia) {
    dst = CountryCode::NC;
    return true;
  }
  else if (src == country_code::New_Zealand) {
    dst = CountryCode::NZ;
    return true;
  }
  else if (src == country_code::Nicaragua) {
    dst = CountryCode::NI;
    return true;
  }
  else if (src == country_code::Niger) {
    dst = CountryCode::NE;
    return true;
  }
  else if (src == country_code::Nigeria) {
    dst = CountryCode::NG;
    return true;
  }
  else if (src == country_code::Niue) {
    dst = CountryCode::NU;
    return true;
  }
  else if (src == country_code::North_Korea) {
    dst = CountryCode::KP;
    return true;
  }
  else if (src == country_code::Northern_Mariana_Islands) {
    dst = CountryCode::MP;
    return true;
  }
  else if (src == country_code::Norway) {
    dst = CountryCode::NO;
    return true;
  }
  else if (src == country_code::Oman) {
    dst = CountryCode::OM;
    return true;
  }
  else if (src == country_code::Pakistan) {
    dst = CountryCode::PK;
    return true;
  }
  else if (src == country_code::Palau) {
    dst = CountryCode::PW;
    return true;
  }
  else if (src == country_code::Palestine) {
    dst = CountryCode::PS;
    return true;
  }
  else if (src == country_code::Panama) {
    dst = CountryCode::PA;
    return true;
  }
  else if (src == country_code::Papua_New_Guinea) {
    dst = CountryCode::PG;
    return true;
  }
  else if (src == country_code::Paraguay) {
    dst = CountryCode::PY;
    return true;
  }
  else if (src == country_code::Peru) {
    dst = CountryCode::PE;
    return true;
  }
  else if (src == country_code::Philippines) {
    dst = CountryCode::PH;
    return true;
  }
  else if (src == country_code::Pitcairn) {
    dst = CountryCode::PN;
    return true;
  }
  else if (src == country_code::Poland) {
    dst = CountryCode::PL;
    return true;
  }
  else if (src == country_code::Portugal) {
    dst = CountryCode::PT;
    return true;
  }
  else if (src == country_code::Puerto_Rico) {
    dst = CountryCode::PR;
    return true;
  }
  else if (src == country_code::Qatar) {
    dst = CountryCode::QA;
    return true;
  }
  else if (src == country_code::Republic_of_the_Congo) {
    dst = CountryCode::CG;
    return true;
  }
  else if (src == country_code::Reunion) {
    dst = CountryCode::RE;
    return true;
  }
  else if (src == country_code::Romania) {
    dst = CountryCode::RO;
    return true;
  }
  else if (src == country_code::Russi) {
    dst = CountryCode::RU;
    return true;
  }
  else if (src == country_code::Rwanda) {
    dst = CountryCode::RW;
    return true;
  }
  else if (src == country_code::Saint_Barthelemy) {
    dst = CountryCode::BL;
    return true;
  }
  else if (src == country_code::Saint_Helena) {
    dst = CountryCode::SH;
    return true;
  }
  else if (src == country_code::Saint_Kitts_and_Nevis) {
    dst = CountryCode::KN;
    return true;
  }
  else if (src == country_code::Saint_Lucia) {
    dst = CountryCode::LC;
    return true;
  }
  else if (src == country_code::Saint_Martin) {
    dst = CountryCode::MF;
    return true;
  }
  else if (src == country_code::Saint_Pierre_and_Miquelon) {
    dst = CountryCode::PM;
    return true;
  }
  else if (src == country_code::Saint_Vincent_and_the_Grenadines) {
    dst = CountryCode::VC;
    return true;
  }
  else if (src == country_code::Samoa) {
    dst = CountryCode::WS;
    return true;
  }
  else if (src == country_code::San_Marino) {
    dst = CountryCode::SM;
    return true;
  }
  else if (src == country_code::Sao_Tome_and_Principe) {
    dst = CountryCode::ST;
    return true;
  }
  else if (src == country_code::Saudi_Arabia) {
    dst = CountryCode::SA;
    return true;
  }
  else if (src == country_code::Senegal) {
    dst = CountryCode::SN;
    return true;
  }
  else if (src == country_code::Serbia) {
    dst = CountryCode::RS;
    return true;
  }
  else if (src == country_code::Seychelles) {
    dst = CountryCode::SC;
    return true;
  }
  else if (src == country_code::Sierra_Leone) {
    dst = CountryCode::SL;
    return true;
  }
  else if (src == country_code::Singapore) {
    dst = CountryCode::SG;
    return true;
  }
  else if (src == country_code::Sint_Maarten) {
    dst = CountryCode::SX;
    return true;
  }
  else if (src == country_code::Slovakia) {
    dst = CountryCode::SK;
    return true;
  }
  else if (src == country_code::Slovenia) {
    dst = CountryCode::SI;
    return true;
  }
  else if (src == country_code::Solomon_Islands) {
    dst = CountryCode::SB;
    return true;
  }
  else if (src == country_code::Somalia) {
    dst = CountryCode::SO;
    return true;
  }
  else if (src == country_code::South_Africa) {
    dst = CountryCode::ZA;
    return true;
  }
  else if (src == country_code::South_Korea) {
    dst = CountryCode::KR;
    return true;
  }
  else if (src == country_code::South_Sudan) {
    dst = CountryCode::SS;
    return true;
  }
  else if (src == country_code::Spain) {
    dst = CountryCode::ES;
    return true;
  }
  else if (src == country_code::Sri_Lanka) {
    dst = CountryCode::LK;
    return true;
  }
  else if (src == country_code::Sudan) {
    dst = CountryCode::SD;
    return true;
  }
  else if (src == country_code::Suriname) {
    dst = CountryCode::SR;
    return true;
  }
  else if (src == country_code::Svalbard_and_Jan_Mayen) {
    dst = CountryCode::SJ;
    return true;
  }
  else if (src == country_code::Swaziland) {
    dst = CountryCode::SZ;
    return true;
  }
  else if (src == country_code::Sweden) {
    dst = CountryCode::SE;
    return true;
  }
  else if (src == country_code::Switzerland) {
    dst = CountryCode::CH;
    return true;
  }
  else if (src == country_code::Syria) {
    dst = CountryCode::SY;
    return true;
  }
  else if (src == country_code::Taiwan) {
    dst = CountryCode::TW;
    return true;
  }
  else if (src == country_code::Tajikistan) {
    dst = CountryCode::TJ;
    return true;
  }
  else if (src == country_code::Tanzania) {
    dst = CountryCode::TZ;
    return true;
  }
  else if (src == country_code::Thailand) {
    dst = CountryCode::TH;
    return true;
  }
  else if (src == country_code::Togo) {
    dst = CountryCode::TG;
    return true;
  }
  else if (src == country_code::Tokelau) {
    dst = CountryCode::TK;
    return true;
  }
  else if (src == country_code::Tonga) {
    dst = CountryCode::TO;
    return true;
  }
  else if (src == country_code::Trinidad_and_Tobago) {
    dst = CountryCode::TT;
    return true;
  }
  else if (src == country_code::Tunisia) {
    dst = CountryCode::TN;
    return true;
  }
  else if (src == country_code::Turkey) {
    dst = CountryCode::TR;
    return true;
  }
  else if (src == country_code::Turkmenistan) {
    dst = CountryCode::TM;
    return true;
  }
  else if (src == country_code::Turks_and_Caicos_Islands) {
    dst = CountryCode::TC;
    return true;
  }
  else if (src == country_code::Tuvalu) {
    dst = CountryCode::TV;
    return true;
  }
  else if (src == country_code::US_Virgin_Islands) {
    dst = CountryCode::VI;
    return true;
  }
  else if (src == country_code::Uganda) {
    dst = CountryCode::UG;
    return true;
  }
  else if (src == country_code::Ukraine) {
    dst = CountryCode::UA;
    return true;
  }
  else if (src == country_code::United_Arab_Emirates) {
    dst = CountryCode::AE;
    return true;
  }
  else if (src == country_code::United_Kingdom) {
    dst = CountryCode::GB;
    return true;
  }
  else if (src == country_code::United_State) {
    dst = CountryCode::US;
    return true;
  }
  else if (src == country_code::Uruguay) {
    dst = CountryCode::UY;
    return true;
  }
  else if (src == country_code::Uzbekistan) {
    dst = CountryCode::UZ;
    return true;
  }
  else if (src == country_code::Vanuatu) {
    dst = CountryCode::VU;
    return true;
  }
  else if (src == country_code::Vatican) {
    dst = CountryCode::VA;
    return true;
  }
  else if (src == country_code::Venezuela) {
    dst = CountryCode::VE;
    return true;
  }
  else if (src == country_code::Vietnam) {
    dst = CountryCode::VN;
    return true;
  }
  else if (src == country_code::Wallis_and_Futuna) {
    dst = CountryCode::WF;
    return true;
  }
  else if (src == country_code::Western_Sahara) {
    dst = CountryCode::EH;
    return true;
  }
  else if (src == country_code::Yemen) {
    dst = CountryCode::YE;
    return true;
  }
  else if (src == country_code::Zambia) {
    dst = CountryCode::ZM;
    return true;
  }
  else if (src == country_code::Zimbabwe) {
    dst = CountryCode::ZW;
    return true;
  }
  else {
    cerr << "Invalid country code: " << src << endl;
    return false;
  }

  return true;
}

bool WpaSupplicantParser::parseKeyManagement(const string& src, KeyManagement& dst)
{
  if (src == key_mgmt::WPA_PSK) {
    dst = KeyManagement::WPA_PSK;
  }
  else if (src == key_mgmt::WPA_EAP) {
    dst = KeyManagement::WPA_EAP;
  }
  else {
    cerr << "Invalid key management method: " << src << endl;
    return false;
  }

  return true;
}

const char* WpaSupplicantParser::countryCodeToString(CountryCode cc)
{
  switch (cc) {
    case CountryCode::AF:
      return country_code::Afghanistan;
    case CountryCode::AL:
      return country_code::Albania;
    case CountryCode::DZ:
      return country_code::Algeria;
    case CountryCode::AS:
      return country_code::American_Samoa;
    case CountryCode::AD:
      return country_code::Andorra;
    case CountryCode::AO:
      return country_code::Angola;
    case CountryCode::AI:
      return country_code::Anguilla;
    case CountryCode::AQ:
      return country_code::Antarctica;
    case CountryCode::AG:
      return country_code::Antigua_and_Barbuda;
    case CountryCode::AR:
      return country_code::Argentina;
    case CountryCode::AM:
      return country_code::Armenia;
    case CountryCode::AW:
      return country_code::Aruba;
    case CountryCode::AU:
      return country_code::Australia;
    case CountryCode::AT:
      return country_code::Austria;
    case CountryCode::AZ:
      return country_code::Azerbaijan;
    case CountryCode::BS:
      return country_code::Bahamas;
    case CountryCode::BH:
      return country_code::Bahrain;
    case CountryCode::BD:
      return country_code::Bangladesh;
    case CountryCode::BB:
      return country_code::Barbados;
    case CountryCode::BY:
      return country_code::Belarus;
    case CountryCode::BE:
      return country_code::Belgium;
    case CountryCode::BZ:
      return country_code::Belize;
    case CountryCode::BJ:
      return country_code::Benin;
    case CountryCode::BM:
      return country_code::Bermuda;
    case CountryCode::BT:
      return country_code::Bhutan;
    case CountryCode::BO:
      return country_code::Bolivia;
    case CountryCode::BA:
      return country_code::Bosnia_and_Herzegovina;
    case CountryCode::BW:
      return country_code::Botswana;
    case CountryCode::BR:
      return country_code::Brazil;
    case CountryCode::IO:
      return country_code::British_Indian_OceanTerritory;
    case CountryCode::VG:
      return country_code::British_Virgin_Islands;
    case CountryCode::BN:
      return country_code::Brunei;
    case CountryCode::BG:
      return country_code::Bulgaria;
    case CountryCode::BF:
      return country_code::Burkina_Faso;
    case CountryCode::BI:
      return country_code::Burundi;
    case CountryCode::KH:
      return country_code::Cambodia;
    case CountryCode::CM:
      return country_code::Cameroon;
    case CountryCode::CA:
      return country_code::Canad;
    case CountryCode::CV:
      return country_code::Cape_Verde;
    case CountryCode::KY:
      return country_code::Cayman_Islands;
    case CountryCode::CF:
      return country_code::Central_African_Republic;
    case CountryCode::TD:
      return country_code::Chad;
    case CountryCode::CL:
      return country_code::Chile;
    case CountryCode::CN:
      return country_code::China;
    case CountryCode::CX:
      return country_code::Christmas_Island;
    case CountryCode::CC:
      return country_code::Cocos_Islands;
    case CountryCode::CO:
      return country_code::Colombia;
    case CountryCode::KM:
      return country_code::Comoros;
    case CountryCode::CK:
      return country_code::Cook_Islands;
    case CountryCode::CR:
      return country_code::Costa_Rica;
    case CountryCode::HR:
      return country_code::Croatia;
    case CountryCode::CU:
      return country_code::Cuba;
    case CountryCode::CW:
      return country_code::Curacao;
    case CountryCode::CY:
      return country_code::Cyprus;
    case CountryCode::CZ:
      return country_code::Czech_Republic;
    case CountryCode::CD:
      return country_code::Democratic_Republic_of_the_Congo;
    case CountryCode::DK:
      return country_code::Denmark;
    case CountryCode::DJ:
      return country_code::Djibouti;
    case CountryCode::DM:
      return country_code::Dominica;
    case CountryCode::DO:
      return country_code::Dominican_Republic;
    case CountryCode::TL:
      return country_code::East_Timor;
    case CountryCode::EC:
      return country_code::Ecuador;
    case CountryCode::EG:
      return country_code::Egypt;
    case CountryCode::SV:
      return country_code::El_Salvador;
    case CountryCode::GQ:
      return country_code::Equatorial_Guinea;
    case CountryCode::ER:
      return country_code::Eritrea;
    case CountryCode::EE:
      return country_code::Estonia;
    case CountryCode::ET:
      return country_code::Ethiopia;
    case CountryCode::FK:
      return country_code::Falkland_Islands;
    case CountryCode::FO:
      return country_code::Faroe_Islands;
    case CountryCode::FJ:
      return country_code::Fiji;
    case CountryCode::FI:
      return country_code::Finland;
    case CountryCode::FR:
      return country_code::France;
    case CountryCode::PF:
      return country_code::French_Polynesia;
    case CountryCode::GA:
      return country_code::Gabon;
    case CountryCode::GM:
      return country_code::Gambia;
    case CountryCode::GE:
      return country_code::Georgia;
    case CountryCode::DE:
      return country_code::Germany;
    case CountryCode::GH:
      return country_code::Ghana;
    case CountryCode::GI:
      return country_code::Gibraltar;
    case CountryCode::GR:
      return country_code::Greece;
    case CountryCode::GL:
      return country_code::Greenland;
    case CountryCode::GD:
      return country_code::Grenada;
    case CountryCode::GU:
      return country_code::Guam;
    case CountryCode::GT:
      return country_code::Guatemala;
    case CountryCode::GG:
      return country_code::Guernsey;
    case CountryCode::GN:
      return country_code::Guinea;
    case CountryCode::GW:
      return country_code::Guinea_Bissau;
    case CountryCode::GY:
      return country_code::Guyana;
    case CountryCode::HT:
      return country_code::Haiti;
    case CountryCode::HN:
      return country_code::Honduras;
    case CountryCode::HK:
      return country_code::Hong_Kong;
    case CountryCode::HU:
      return country_code::Hungary;
    case CountryCode::IS:
      return country_code::Iceland;
    case CountryCode::IN:
      return country_code::India;
    case CountryCode::ID:
      return country_code::Indonesia;
    case CountryCode::IR:
      return country_code::Iran;
    case CountryCode::IQ:
      return country_code::Iraq;
    case CountryCode::IE:
      return country_code::Ireland;
    case CountryCode::IM:
      return country_code::Isle_of_Man;
    case CountryCode::IL:
      return country_code::Israel;
    case CountryCode::IT:
      return country_code::Italy;
    case CountryCode::CI:
      return country_code::Ivory_Coast;
    case CountryCode::JM:
      return country_code::Jamaica;
    case CountryCode::JP:
      return country_code::Japan;
    case CountryCode::JE:
      return country_code::Jersey;
    case CountryCode::JO:
      return country_code::Jordan;
    case CountryCode::KZ:
      return country_code::Kazakhsta;
    case CountryCode::KE:
      return country_code::Kenya;
    case CountryCode::KI:
      return country_code::Kiribati;
    case CountryCode::XK:
      return country_code::Kosovo;
    case CountryCode::KW:
      return country_code::Kuwait;
    case CountryCode::KG:
      return country_code::Kyrgyzstan;
    case CountryCode::LA:
      return country_code::Laos;
    case CountryCode::LV:
      return country_code::Latvia;
    case CountryCode::LB:
      return country_code::Lebanon;
    case CountryCode::LS:
      return country_code::Lesotho;
    case CountryCode::LR:
      return country_code::Liberia;
    case CountryCode::LY:
      return country_code::Libya;
    case CountryCode::LI:
      return country_code::Liechtenstein;
    case CountryCode::LT:
      return country_code::Lithuania;
    case CountryCode::LU:
      return country_code::Luxembourg;
    case CountryCode::MO:
      return country_code::Macau;
    case CountryCode::MK:
      return country_code::Macedonia;
    case CountryCode::MG:
      return country_code::Madagascar;
    case CountryCode::MW:
      return country_code::Malawi;
    case CountryCode::MY:
      return country_code::Malaysia;
    case CountryCode::MV:
      return country_code::Maldives;
    case CountryCode::ML:
      return country_code::Mali;
    case CountryCode::MT:
      return country_code::Malta;
    case CountryCode::MH:
      return country_code::Marshall_Islands;
    case CountryCode::MR:
      return country_code::Mauritania;
    case CountryCode::MU:
      return country_code::Mauritius;
    case CountryCode::YT:
      return country_code::Mayotte;
    case CountryCode::MX:
      return country_code::Mexico;
    case CountryCode::FM:
      return country_code::Micronesia;
    case CountryCode::MD:
      return country_code::Moldova;
    case CountryCode::MC:
      return country_code::Monaco;
    case CountryCode::MN:
      return country_code::Mongolia;
    case CountryCode::ME:
      return country_code::Montenegro;
    case CountryCode::MS:
      return country_code::Montserrat;
    case CountryCode::MA:
      return country_code::Morocco;
    case CountryCode::MZ:
      return country_code::Mozambique;
    case CountryCode::MM:
      return country_code::Myanmar;
    case CountryCode::NA:
      return country_code::Namibia;
    case CountryCode::NR:
      return country_code::Nauru;
    case CountryCode::NP:
      return country_code::Nepal;
    case CountryCode::NL:
      return country_code::Netherlands;
    case CountryCode::AN:
      return country_code::Netherlands_Antilles;
    case CountryCode::NC:
      return country_code::New_Caledonia;
    case CountryCode::NZ:
      return country_code::New_Zealand;
    case CountryCode::NI:
      return country_code::Nicaragua;
    case CountryCode::NE:
      return country_code::Niger;
    case CountryCode::NG:
      return country_code::Nigeria;
    case CountryCode::NU:
      return country_code::Niue;
    case CountryCode::KP:
      return country_code::North_Korea;
    case CountryCode::MP:
      return country_code::Northern_Mariana_Islands;
    case CountryCode::NO:
      return country_code::Norway;
    case CountryCode::OM:
      return country_code::Oman;
    case CountryCode::PK:
      return country_code::Pakistan;
    case CountryCode::PW:
      return country_code::Palau;
    case CountryCode::PS:
      return country_code::Palestine;
    case CountryCode::PA:
      return country_code::Panama;
    case CountryCode::PG:
      return country_code::Papua_New_Guinea;
    case CountryCode::PY:
      return country_code::Paraguay;
    case CountryCode::PE:
      return country_code::Peru;
    case CountryCode::PH:
      return country_code::Philippines;
    case CountryCode::PN:
      return country_code::Pitcairn;
    case CountryCode::PL:
      return country_code::Poland;
    case CountryCode::PT:
      return country_code::Portugal;
    case CountryCode::PR:
      return country_code::Puerto_Rico;
    case CountryCode::QA:
      return country_code::Qatar;
    case CountryCode::CG:
      return country_code::Republic_of_the_Congo;
    case CountryCode::RE:
      return country_code::Reunion;
    case CountryCode::RO:
      return country_code::Romania;
    case CountryCode::RU:
      return country_code::Russi;
    case CountryCode::RW:
      return country_code::Rwanda;
    case CountryCode::BL:
      return country_code::Saint_Barthelemy;
    case CountryCode::SH:
      return country_code::Saint_Helena;
    case CountryCode::KN:
      return country_code::Saint_Kitts_and_Nevis;
    case CountryCode::LC:
      return country_code::Saint_Lucia;
    case CountryCode::MF:
      return country_code::Saint_Martin;
    case CountryCode::PM:
      return country_code::Saint_Pierre_and_Miquelon;
    case CountryCode::VC:
      return country_code::Saint_Vincent_and_the_Grenadines;
    case CountryCode::WS:
      return country_code::Samoa;
    case CountryCode::SM:
      return country_code::San_Marino;
    case CountryCode::ST:
      return country_code::Sao_Tome_and_Principe;
    case CountryCode::SA:
      return country_code::Saudi_Arabia;
    case CountryCode::SN:
      return country_code::Senegal;
    case CountryCode::RS:
      return country_code::Serbia;
    case CountryCode::SC:
      return country_code::Seychelles;
    case CountryCode::SL:
      return country_code::Sierra_Leone;
    case CountryCode::SG:
      return country_code::Singapore;
    case CountryCode::SX:
      return country_code::Sint_Maarten;
    case CountryCode::SK:
      return country_code::Slovakia;
    case CountryCode::SI:
      return country_code::Slovenia;
    case CountryCode::SB:
      return country_code::Solomon_Islands;
    case CountryCode::SO:
      return country_code::Somalia;
    case CountryCode::ZA:
      return country_code::South_Africa;
    case CountryCode::KR:
      return country_code::South_Korea;
    case CountryCode::SS:
      return country_code::South_Sudan;
    case CountryCode::ES:
      return country_code::Spain;
    case CountryCode::LK:
      return country_code::Sri_Lanka;
    case CountryCode::SD:
      return country_code::Sudan;
    case CountryCode::SR:
      return country_code::Suriname;
    case CountryCode::SJ:
      return country_code::Svalbard_and_Jan_Mayen;
    case CountryCode::SZ:
      return country_code::Swaziland;
    case CountryCode::SE:
      return country_code::Sweden;
    case CountryCode::CH:
      return country_code::Switzerland;
    case CountryCode::SY:
      return country_code::Syria;
    case CountryCode::TW:
      return country_code::Taiwan;
    case CountryCode::TJ:
      return country_code::Tajikistan;
    case CountryCode::TZ:
      return country_code::Tanzania;
    case CountryCode::TH:
      return country_code::Thailand;
    case CountryCode::TG:
      return country_code::Togo;
    case CountryCode::TK:
      return country_code::Tokelau;
    case CountryCode::TO:
      return country_code::Tonga;
    case CountryCode::TT:
      return country_code::Trinidad_and_Tobago;
    case CountryCode::TN:
      return country_code::Tunisia;
    case CountryCode::TR:
      return country_code::Turkey;
    case CountryCode::TM:
      return country_code::Turkmenistan;
    case CountryCode::TC:
      return country_code::Turks_and_Caicos_Islands;
    case CountryCode::TV:
      return country_code::Tuvalu;
    case CountryCode::VI:
      return country_code::US_Virgin_Islands;
    case CountryCode::UG:
      return country_code::Uganda;
    case CountryCode::UA:
      return country_code::Ukraine;
    case CountryCode::AE:
      return country_code::United_Arab_Emirates;
    case CountryCode::GB:
      return country_code::United_Kingdom;
    case CountryCode::US:
      return country_code::United_State;
    case CountryCode::UY:
      return country_code::Uruguay;
    case CountryCode::UZ:
      return country_code::Uzbekistan;
    case CountryCode::VU:
      return country_code::Vanuatu;
    case CountryCode::VA:
      return country_code::Vatican;
    case CountryCode::VE:
      return country_code::Venezuela;
    case CountryCode::VN:
      return country_code::Vietnam;
    case CountryCode::WF:
      return country_code::Wallis_and_Futuna;
    case CountryCode::EH:
      return country_code::Western_Sahara;
    case CountryCode::YE:
      return country_code::Yemen;
    case CountryCode::ZM:
      return country_code::Zambia;
    case CountryCode::ZW:
      return country_code::Zimbabwe;
    default:
      throw;
  }
}

const char* WpaSupplicantParser::keyManagementToString(KeyManagement key_mgmt)
{
  switch (key_mgmt) {
    case KeyManagement::WPA_PSK:
      return key_mgmt::WPA_PSK;
    case KeyManagement::WPA_EAP:
      return key_mgmt::WPA_EAP;
    default:
      throw;
  }
}
}  // namespace wpa
