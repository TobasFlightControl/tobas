#include "tobas_wpa_supplicant/parse.hpp"

#include <iostream>

#include <tobas_string_tools/core.hpp>

#include "tobas_wpa_supplicant/constants.hpp"

namespace tobas
{
namespace wpa
{
Parser::Parser()
{
}

bool Parser::parseFromText(const std::string& text, Data& dst)
{
  // Reset
  dst.networks.clear();

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
      if (!parseCountryCode(country_str, dst.country)) {
        return false;
      }
      continue;
    }

    // ctrl_interface
    if (line.starts_with(kCtrlInterfacePrefix)) {
      dst.ctrl_interface = line.substr(sizeof(kCtrlInterfacePrefix) - 1);
      continue;
    }

    // update_config
    if (line.starts_with(kUpdateConfigPrefix)) {
      dst.update_config = (line.substr(sizeof(kUpdateConfigPrefix) - 1) == "1");
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
        std::cerr << "Unexpected closing bracket." << std::endl;
        return false;
      }
      dst.networks.push_back(network);
      in_network_block = false;
      continue;
    }

    // ssid
    if (line.starts_with(kSSIDPrefix)) {
      if (!in_network_block) {
        std::cerr << "SSID setting found outside network block." << std::endl;
        return false;
      }
      network.ssid = str::stripQuates(line.substr(sizeof(kSSIDPrefix) - 1));
      continue;
    }

    // psk
    if (line.starts_with(kPSKPrefix)) {
      if (!in_network_block) {
        std::cerr << "PSK setting found outside network block." << std::endl;
        return false;
      }
      network.psk = str::stripQuates(line.substr(sizeof(kPSKPrefix) - 1));
      continue;
    }

    // key_mgmt
    if (line.starts_with(kKeyMgmtPrefix)) {
      if (!in_network_block) {
        std::cerr << "Key management setting found outside network block." << std::endl;
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
        std::cerr << "Priority setting found outside network block." << std::endl;
        return false;
      }
      network.priority = stoi(line.substr(sizeof(kPriorityPrefix) - 1));
      continue;
    }
  }

  return true;
}

bool Parser::parseCountryCode(const std::string& src, CountryCode& dst)
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
    std::cerr << "Invalid country code: " << src << std::endl;
    return false;
  }

  return true;
}

bool Parser::parseKeyManagement(const std::string& src, KeyManagement& dst)
{
  if (src == key_mgmt::WPA_PSK) {
    dst = KeyManagement::WPA_PSK;
  }
  else if (src == key_mgmt::WPA_EAP) {
    dst = KeyManagement::WPA_EAP;
  }
  else {
    std::cerr << "Invalid key management method: " << src << std::endl;
    return false;
  }

  return true;
}
}  // namespace wpa
}  // namespace tobas
