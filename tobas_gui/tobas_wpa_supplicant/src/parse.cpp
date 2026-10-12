// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_wpa_supplicant/parse.hpp"

#include <expected>
#include <stdexcept>

#include <tobas_string_tools/core.hpp>

#include "tobas_wpa_supplicant/constants.hpp"

namespace tobas
{
namespace wpa
{
namespace
{
std::expected<CountryCode, std::string> parseCountryCode(const std::string& _src)
{
  if (_src == country_code::Afghanistan) {
    return CountryCode::AF;
  }
  else if (_src == country_code::Albania) {
    return CountryCode::AL;
  }
  else if (_src == country_code::Algeria) {
    return CountryCode::DZ;
  }
  else if (_src == country_code::American_Samoa) {
    return CountryCode::AS;
  }
  else if (_src == country_code::Andorra) {
    return CountryCode::AD;
  }
  else if (_src == country_code::Angola) {
    return CountryCode::AO;
  }
  else if (_src == country_code::Anguilla) {
    return CountryCode::AI;
  }
  else if (_src == country_code::Antarctica) {
    return CountryCode::AQ;
  }
  else if (_src == country_code::Antigua_and_Barbuda) {
    return CountryCode::AG;
  }
  else if (_src == country_code::Argentina) {
    return CountryCode::AR;
  }
  else if (_src == country_code::Armenia) {
    return CountryCode::AM;
  }
  else if (_src == country_code::Aruba) {
    return CountryCode::AW;
  }
  else if (_src == country_code::Australia) {
    return CountryCode::AU;
  }
  else if (_src == country_code::Austria) {
    return CountryCode::AT;
  }
  else if (_src == country_code::Azerbaijan) {
    return CountryCode::AZ;
  }
  else if (_src == country_code::Bahamas) {
    return CountryCode::BS;
  }
  else if (_src == country_code::Bahrain) {
    return CountryCode::BH;
  }
  else if (_src == country_code::Bangladesh) {
    return CountryCode::BD;
  }
  else if (_src == country_code::Barbados) {
    return CountryCode::BB;
  }
  else if (_src == country_code::Belarus) {
    return CountryCode::BY;
  }
  else if (_src == country_code::Belgium) {
    return CountryCode::BE;
  }
  else if (_src == country_code::Belize) {
    return CountryCode::BZ;
  }
  else if (_src == country_code::Benin) {
    return CountryCode::BJ;
  }
  else if (_src == country_code::Bermuda) {
    return CountryCode::BM;
  }
  else if (_src == country_code::Bhutan) {
    return CountryCode::BT;
  }
  else if (_src == country_code::Bolivia) {
    return CountryCode::BO;
  }
  else if (_src == country_code::Bosnia_and_Herzegovina) {
    return CountryCode::BA;
  }
  else if (_src == country_code::Botswana) {
    return CountryCode::BW;
  }
  else if (_src == country_code::Brazil) {
    return CountryCode::BR;
  }
  else if (_src == country_code::British_Indian_OceanTerritory) {
    return CountryCode::IO;
  }
  else if (_src == country_code::British_Virgin_Islands) {
    return CountryCode::VG;
  }
  else if (_src == country_code::Brunei) {
    return CountryCode::BN;
  }
  else if (_src == country_code::Bulgaria) {
    return CountryCode::BG;
  }
  else if (_src == country_code::Burkina_Faso) {
    return CountryCode::BF;
  }
  else if (_src == country_code::Burundi) {
    return CountryCode::BI;
  }
  else if (_src == country_code::Cambodia) {
    return CountryCode::KH;
  }
  else if (_src == country_code::Cameroon) {
    return CountryCode::CM;
  }
  else if (_src == country_code::Canad) {
    return CountryCode::CA;
  }
  else if (_src == country_code::Cape_Verde) {
    return CountryCode::CV;
  }
  else if (_src == country_code::Cayman_Islands) {
    return CountryCode::KY;
  }
  else if (_src == country_code::Central_African_Republic) {
    return CountryCode::CF;
  }
  else if (_src == country_code::Chad) {
    return CountryCode::TD;
  }
  else if (_src == country_code::Chile) {
    return CountryCode::CL;
  }
  else if (_src == country_code::China) {
    return CountryCode::CN;
  }
  else if (_src == country_code::Christmas_Island) {
    return CountryCode::CX;
  }
  else if (_src == country_code::Cocos_Islands) {
    return CountryCode::CC;
  }
  else if (_src == country_code::Colombia) {
    return CountryCode::CO;
  }
  else if (_src == country_code::Comoros) {
    return CountryCode::KM;
  }
  else if (_src == country_code::Cook_Islands) {
    return CountryCode::CK;
  }
  else if (_src == country_code::Costa_Rica) {
    return CountryCode::CR;
  }
  else if (_src == country_code::Croatia) {
    return CountryCode::HR;
  }
  else if (_src == country_code::Cuba) {
    return CountryCode::CU;
  }
  else if (_src == country_code::Curacao) {
    return CountryCode::CW;
  }
  else if (_src == country_code::Cyprus) {
    return CountryCode::CY;
  }
  else if (_src == country_code::Czech_Republic) {
    return CountryCode::CZ;
  }
  else if (_src == country_code::Democratic_Republic_of_the_Congo) {
    return CountryCode::CD;
  }
  else if (_src == country_code::Denmark) {
    return CountryCode::DK;
  }
  else if (_src == country_code::Djibouti) {
    return CountryCode::DJ;
  }
  else if (_src == country_code::Dominica) {
    return CountryCode::DM;
  }
  else if (_src == country_code::Dominican_Republic) {
    return CountryCode::DO;
  }
  else if (_src == country_code::East_Timor) {
    return CountryCode::TL;
  }
  else if (_src == country_code::Ecuador) {
    return CountryCode::EC;
  }
  else if (_src == country_code::Egypt) {
    return CountryCode::EG;
  }
  else if (_src == country_code::El_Salvador) {
    return CountryCode::SV;
  }
  else if (_src == country_code::Equatorial_Guinea) {
    return CountryCode::GQ;
  }
  else if (_src == country_code::Eritrea) {
    return CountryCode::ER;
  }
  else if (_src == country_code::Estonia) {
    return CountryCode::EE;
  }
  else if (_src == country_code::Ethiopia) {
    return CountryCode::ET;
  }
  else if (_src == country_code::Falkland_Islands) {
    return CountryCode::FK;
  }
  else if (_src == country_code::Faroe_Islands) {
    return CountryCode::FO;
  }
  else if (_src == country_code::Fiji) {
    return CountryCode::FJ;
  }
  else if (_src == country_code::Finland) {
    return CountryCode::FI;
  }
  else if (_src == country_code::France) {
    return CountryCode::FR;
  }
  else if (_src == country_code::French_Polynesia) {
    return CountryCode::PF;
  }
  else if (_src == country_code::Gabon) {
    return CountryCode::GA;
  }
  else if (_src == country_code::Gambia) {
    return CountryCode::GM;
  }
  else if (_src == country_code::Georgia) {
    return CountryCode::GE;
  }
  else if (_src == country_code::Germany) {
    return CountryCode::DE;
  }
  else if (_src == country_code::Ghana) {
    return CountryCode::GH;
  }
  else if (_src == country_code::Gibraltar) {
    return CountryCode::GI;
  }
  else if (_src == country_code::Greece) {
    return CountryCode::GR;
  }
  else if (_src == country_code::Greenland) {
    return CountryCode::GL;
  }
  else if (_src == country_code::Grenada) {
    return CountryCode::GD;
  }
  else if (_src == country_code::Guam) {
    return CountryCode::GU;
  }
  else if (_src == country_code::Guatemala) {
    return CountryCode::GT;
  }
  else if (_src == country_code::Guernsey) {
    return CountryCode::GG;
  }
  else if (_src == country_code::Guinea) {
    return CountryCode::GN;
  }
  else if (_src == country_code::Guinea_Bissau) {
    return CountryCode::GW;
  }
  else if (_src == country_code::Guyana) {
    return CountryCode::GY;
  }
  else if (_src == country_code::Haiti) {
    return CountryCode::HT;
  }
  else if (_src == country_code::Honduras) {
    return CountryCode::HN;
  }
  else if (_src == country_code::Hong_Kong) {
    return CountryCode::HK;
  }
  else if (_src == country_code::Hungary) {
    return CountryCode::HU;
  }
  else if (_src == country_code::Iceland) {
    return CountryCode::IS;
  }
  else if (_src == country_code::India) {
    return CountryCode::IN;
  }
  else if (_src == country_code::Indonesia) {
    return CountryCode::ID;
  }
  else if (_src == country_code::Iran) {
    return CountryCode::IR;
  }
  else if (_src == country_code::Iraq) {
    return CountryCode::IQ;
  }
  else if (_src == country_code::Ireland) {
    return CountryCode::IE;
  }
  else if (_src == country_code::Isle_of_Man) {
    return CountryCode::IM;
  }
  else if (_src == country_code::Israel) {
    return CountryCode::IL;
  }
  else if (_src == country_code::Italy) {
    return CountryCode::IT;
  }
  else if (_src == country_code::Ivory_Coast) {
    return CountryCode::CI;
  }
  else if (_src == country_code::Jamaica) {
    return CountryCode::JM;
  }
  else if (_src == country_code::Japan) {
    return CountryCode::JP;
  }
  else if (_src == country_code::Jersey) {
    return CountryCode::JE;
  }
  else if (_src == country_code::Jordan) {
    return CountryCode::JO;
  }
  else if (_src == country_code::Kazakhsta) {
    return CountryCode::KZ;
  }
  else if (_src == country_code::Kenya) {
    return CountryCode::KE;
  }
  else if (_src == country_code::Kiribati) {
    return CountryCode::KI;
  }
  else if (_src == country_code::Kosovo) {
    return CountryCode::XK;
  }
  else if (_src == country_code::Kuwait) {
    return CountryCode::KW;
  }
  else if (_src == country_code::Kyrgyzstan) {
    return CountryCode::KG;
  }
  else if (_src == country_code::Laos) {
    return CountryCode::LA;
  }
  else if (_src == country_code::Latvia) {
    return CountryCode::LV;
  }
  else if (_src == country_code::Lebanon) {
    return CountryCode::LB;
  }
  else if (_src == country_code::Lesotho) {
    return CountryCode::LS;
  }
  else if (_src == country_code::Liberia) {
    return CountryCode::LR;
  }
  else if (_src == country_code::Libya) {
    return CountryCode::LY;
  }
  else if (_src == country_code::Liechtenstein) {
    return CountryCode::LI;
  }
  else if (_src == country_code::Lithuania) {
    return CountryCode::LT;
  }
  else if (_src == country_code::Luxembourg) {
    return CountryCode::LU;
  }
  else if (_src == country_code::Macau) {
    return CountryCode::MO;
  }
  else if (_src == country_code::Macedonia) {
    return CountryCode::MK;
  }
  else if (_src == country_code::Madagascar) {
    return CountryCode::MG;
  }
  else if (_src == country_code::Malawi) {
    return CountryCode::MW;
  }
  else if (_src == country_code::Malaysia) {
    return CountryCode::MY;
  }
  else if (_src == country_code::Maldives) {
    return CountryCode::MV;
  }
  else if (_src == country_code::Mali) {
    return CountryCode::ML;
  }
  else if (_src == country_code::Malta) {
    return CountryCode::MT;
  }
  else if (_src == country_code::Marshall_Islands) {
    return CountryCode::MH;
  }
  else if (_src == country_code::Mauritania) {
    return CountryCode::MR;
  }
  else if (_src == country_code::Mauritius) {
    return CountryCode::MU;
  }
  else if (_src == country_code::Mayotte) {
    return CountryCode::YT;
  }
  else if (_src == country_code::Mexico) {
    return CountryCode::MX;
  }
  else if (_src == country_code::Micronesia) {
    return CountryCode::FM;
  }
  else if (_src == country_code::Moldova) {
    return CountryCode::MD;
  }
  else if (_src == country_code::Monaco) {
    return CountryCode::MC;
  }
  else if (_src == country_code::Mongolia) {
    return CountryCode::MN;
  }
  else if (_src == country_code::Montenegro) {
    return CountryCode::ME;
  }
  else if (_src == country_code::Montserrat) {
    return CountryCode::MS;
  }
  else if (_src == country_code::Morocco) {
    return CountryCode::MA;
  }
  else if (_src == country_code::Mozambique) {
    return CountryCode::MZ;
  }
  else if (_src == country_code::Myanmar) {
    return CountryCode::MM;
  }
  else if (_src == country_code::Namibia) {
    return CountryCode::NA;
  }
  else if (_src == country_code::Nauru) {
    return CountryCode::NR;
  }
  else if (_src == country_code::Nepal) {
    return CountryCode::NP;
  }
  else if (_src == country_code::Netherlands) {
    return CountryCode::NL;
  }
  else if (_src == country_code::Netherlands_Antilles) {
    return CountryCode::AN;
  }
  else if (_src == country_code::New_Caledonia) {
    return CountryCode::NC;
  }
  else if (_src == country_code::New_Zealand) {
    return CountryCode::NZ;
  }
  else if (_src == country_code::Nicaragua) {
    return CountryCode::NI;
  }
  else if (_src == country_code::Niger) {
    return CountryCode::NE;
  }
  else if (_src == country_code::Nigeria) {
    return CountryCode::NG;
  }
  else if (_src == country_code::Niue) {
    return CountryCode::NU;
  }
  else if (_src == country_code::North_Korea) {
    return CountryCode::KP;
  }
  else if (_src == country_code::Northern_Mariana_Islands) {
    return CountryCode::MP;
  }
  else if (_src == country_code::Norway) {
    return CountryCode::NO;
  }
  else if (_src == country_code::Oman) {
    return CountryCode::OM;
  }
  else if (_src == country_code::Pakistan) {
    return CountryCode::PK;
  }
  else if (_src == country_code::Palau) {
    return CountryCode::PW;
  }
  else if (_src == country_code::Palestine) {
    return CountryCode::PS;
  }
  else if (_src == country_code::Panama) {
    return CountryCode::PA;
  }
  else if (_src == country_code::Papua_New_Guinea) {
    return CountryCode::PG;
  }
  else if (_src == country_code::Paraguay) {
    return CountryCode::PY;
  }
  else if (_src == country_code::Peru) {
    return CountryCode::PE;
  }
  else if (_src == country_code::Philippines) {
    return CountryCode::PH;
  }
  else if (_src == country_code::Pitcairn) {
    return CountryCode::PN;
  }
  else if (_src == country_code::Poland) {
    return CountryCode::PL;
  }
  else if (_src == country_code::Portugal) {
    return CountryCode::PT;
  }
  else if (_src == country_code::Puerto_Rico) {
    return CountryCode::PR;
  }
  else if (_src == country_code::Qatar) {
    return CountryCode::QA;
  }
  else if (_src == country_code::Republic_of_the_Congo) {
    return CountryCode::CG;
  }
  else if (_src == country_code::Reunion) {
    return CountryCode::RE;
  }
  else if (_src == country_code::Romania) {
    return CountryCode::RO;
  }
  else if (_src == country_code::Russi) {
    return CountryCode::RU;
  }
  else if (_src == country_code::Rwanda) {
    return CountryCode::RW;
  }
  else if (_src == country_code::Saint_Barthelemy) {
    return CountryCode::BL;
  }
  else if (_src == country_code::Saint_Helena) {
    return CountryCode::SH;
  }
  else if (_src == country_code::Saint_Kitts_and_Nevis) {
    return CountryCode::KN;
  }
  else if (_src == country_code::Saint_Lucia) {
    return CountryCode::LC;
  }
  else if (_src == country_code::Saint_Martin) {
    return CountryCode::MF;
  }
  else if (_src == country_code::Saint_Pierre_and_Miquelon) {
    return CountryCode::PM;
  }
  else if (_src == country_code::Saint_Vincent_and_the_Grenadines) {
    return CountryCode::VC;
  }
  else if (_src == country_code::Samoa) {
    return CountryCode::WS;
  }
  else if (_src == country_code::San_Marino) {
    return CountryCode::SM;
  }
  else if (_src == country_code::Sao_Tome_and_Principe) {
    return CountryCode::ST;
  }
  else if (_src == country_code::Saudi_Arabia) {
    return CountryCode::SA;
  }
  else if (_src == country_code::Senegal) {
    return CountryCode::SN;
  }
  else if (_src == country_code::Serbia) {
    return CountryCode::RS;
  }
  else if (_src == country_code::Seychelles) {
    return CountryCode::SC;
  }
  else if (_src == country_code::Sierra_Leone) {
    return CountryCode::SL;
  }
  else if (_src == country_code::Singapore) {
    return CountryCode::SG;
  }
  else if (_src == country_code::Sint_Maarten) {
    return CountryCode::SX;
  }
  else if (_src == country_code::Slovakia) {
    return CountryCode::SK;
  }
  else if (_src == country_code::Slovenia) {
    return CountryCode::SI;
  }
  else if (_src == country_code::Solomon_Islands) {
    return CountryCode::SB;
  }
  else if (_src == country_code::Somalia) {
    return CountryCode::SO;
  }
  else if (_src == country_code::South_Africa) {
    return CountryCode::ZA;
  }
  else if (_src == country_code::South_Korea) {
    return CountryCode::KR;
  }
  else if (_src == country_code::South_Sudan) {
    return CountryCode::SS;
  }
  else if (_src == country_code::Spain) {
    return CountryCode::ES;
  }
  else if (_src == country_code::Sri_Lanka) {
    return CountryCode::LK;
  }
  else if (_src == country_code::Sudan) {
    return CountryCode::SD;
  }
  else if (_src == country_code::Suriname) {
    return CountryCode::SR;
  }
  else if (_src == country_code::Svalbard_and_Jan_Mayen) {
    return CountryCode::SJ;
  }
  else if (_src == country_code::Swaziland) {
    return CountryCode::SZ;
  }
  else if (_src == country_code::Sweden) {
    return CountryCode::SE;
  }
  else if (_src == country_code::Switzerland) {
    return CountryCode::CH;
  }
  else if (_src == country_code::Syria) {
    return CountryCode::SY;
  }
  else if (_src == country_code::Taiwan) {
    return CountryCode::TW;
  }
  else if (_src == country_code::Tajikistan) {
    return CountryCode::TJ;
  }
  else if (_src == country_code::Tanzania) {
    return CountryCode::TZ;
  }
  else if (_src == country_code::Thailand) {
    return CountryCode::TH;
  }
  else if (_src == country_code::Togo) {
    return CountryCode::TG;
  }
  else if (_src == country_code::Tokelau) {
    return CountryCode::TK;
  }
  else if (_src == country_code::Tonga) {
    return CountryCode::TO;
  }
  else if (_src == country_code::Trinidad_and_Tobago) {
    return CountryCode::TT;
  }
  else if (_src == country_code::Tunisia) {
    return CountryCode::TN;
  }
  else if (_src == country_code::Turkey) {
    return CountryCode::TR;
  }
  else if (_src == country_code::Turkmenistan) {
    return CountryCode::TM;
  }
  else if (_src == country_code::Turks_and_Caicos_Islands) {
    return CountryCode::TC;
  }
  else if (_src == country_code::Tuvalu) {
    return CountryCode::TV;
  }
  else if (_src == country_code::US_Virgin_Islands) {
    return CountryCode::VI;
  }
  else if (_src == country_code::Uganda) {
    return CountryCode::UG;
  }
  else if (_src == country_code::Ukraine) {
    return CountryCode::UA;
  }
  else if (_src == country_code::United_Arab_Emirates) {
    return CountryCode::AE;
  }
  else if (_src == country_code::United_Kingdom) {
    return CountryCode::GB;
  }
  else if (_src == country_code::United_State) {
    return CountryCode::US;
  }
  else if (_src == country_code::Uruguay) {
    return CountryCode::UY;
  }
  else if (_src == country_code::Uzbekistan) {
    return CountryCode::UZ;
  }
  else if (_src == country_code::Vanuatu) {
    return CountryCode::VU;
  }
  else if (_src == country_code::Vatican) {
    return CountryCode::VA;
  }
  else if (_src == country_code::Venezuela) {
    return CountryCode::VE;
  }
  else if (_src == country_code::Vietnam) {
    return CountryCode::VN;
  }
  else if (_src == country_code::Wallis_and_Futuna) {
    return CountryCode::WF;
  }
  else if (_src == country_code::Western_Sahara) {
    return CountryCode::EH;
  }
  else if (_src == country_code::Yemen) {
    return CountryCode::YE;
  }
  else if (_src == country_code::Zambia) {
    return CountryCode::ZM;
  }
  else if (_src == country_code::Zimbabwe) {
    return CountryCode::ZW;
  }
  else {
    return std::unexpected("Invalid country code: " + _src);
  }
}

std::expected<int, std::string> parseInteger(const std::string& text, const std::string& field)
{
  try {
    return std::stoi(text);
  }
  catch (const std::invalid_argument&) {
    return std::unexpected("Invalid integer for " + field + ".");
  }
  catch (const std::out_of_range&) {
    return std::unexpected("Integer out of range for " + field + ".");
  }
}
}  // namespace

std::expected<Data, std::string> parseFromText(const std::string& text)
{
  Data res;

  const auto lines = str::splitLines(text);
  Network network;
  bool in_network_block = false;

  for (auto line : lines) {
    if (line.find_first_not_of(" \t\n\r\f\v") == std::string::npos) {
      continue;
    }

    // Remove leading and trailing whitespace.
    line = str::trim(line);

    // Skip empty and comment lines.
    if (line.empty() || line.starts_with('#')) {
      continue;
    }

    // country
    if (line.starts_with(kCountryPrefix)) {
      const auto country = parseCountryCode(line.substr(sizeof(kCountryPrefix) - 1));
      if (!country) {
        return std::unexpected(country.error());
      }
      res.country = *country;
      continue;
    }

    // ctrl_interface
    if (line.starts_with(kCtrlInterfacePrefix)) {
      res.ctrl_interface = line.substr(sizeof(kCtrlInterfacePrefix) - 1);
      continue;
    }

    // update_config
    if (line.starts_with(kUpdateConfigPrefix)) {
      res.update_config = (line.substr(sizeof(kUpdateConfigPrefix) - 1) == "1");
      continue;
    }

    // Start of a network block.
    if (line == kStartNetworkBlock) {
      network = Network();
      in_network_block = true;
      continue;
    }

    // End of a network block.
    if (line == kStopNetworkBlock) {
      if (!in_network_block) {
        return std::unexpected("Unexpected closing bracket.");
      }
      res.networks.push_back(network);
      in_network_block = false;
      continue;
    }

    if (in_network_block) {
      // ssid
      if (line.starts_with(kSsidPrefix)) {
        const auto value = line.substr(sizeof(kSsidPrefix) - 1);
        network.ssid = value.empty() ? value : str::stripQuates(value);
        continue;
      }

      // psk
      if (line.starts_with(kPskPrefix)) {
        const auto value = line.substr(sizeof(kPskPrefix) - 1);
        network.psk = value.empty() ? value : str::stripQuates(value);
        continue;
      }

      // priority
      if (line.starts_with(kPriorityPrefix)) {
        const auto priority = parseInteger(line.substr(sizeof(kPriorityPrefix) - 1), "priority");
        if (!priority) {
          return std::unexpected(priority.error());
        }
        network.priority = *priority;
        continue;
      }

      // scan_ssid
      if (line.starts_with(kScanSsidPrefix)) {
        const auto scan_ssid = parseInteger(line.substr(sizeof(kScanSsidPrefix) - 1), "scan_ssid");
        if (!scan_ssid) {
          return std::unexpected(scan_ssid.error());
        }
        network.scan_ssid = static_cast<bool>(*scan_ssid);
        continue;
      }

      // key_mgmt
      if (line.starts_with(kKeyMgmtPrefix)) {
        const auto key_mgmt_token = line.substr(sizeof(kKeyMgmtPrefix) - 1);
        if (!enumFromToken(key_mgmt_token, network.key_mgmt)) {
          return std::unexpected("Invalid key management token: " + key_mgmt_token);
        }
        continue;
      }

      // sae_password
      if (line.starts_with(kSaePasswordPrefix)) {
        const auto value = line.substr(sizeof(kSaePasswordPrefix) - 1);
        network.psk = value.empty() ? value : str::stripQuates(value);
        continue;
      }
    }
  }

  return res;
}
}  // namespace wpa
}  // namespace tobas
