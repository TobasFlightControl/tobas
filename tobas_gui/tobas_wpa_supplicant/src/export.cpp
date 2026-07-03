// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_wpa_supplicant/export.hpp"

#include <sstream>

#include "tobas_wpa_supplicant/constants.hpp"

namespace tobas
{
namespace wpa
{
Exporter::Exporter()
{
}

std::string Exporter::exportText(const Data& src) const
{
  std::ostringstream oss;

  // Country
  oss << kCountryPrefix << countryCodeToString(src.country) << "\n";

  // ctrl_interface
  oss << kCtrlInterfacePrefix << src.ctrl_interface << "\n";

  // update_config
  oss << kUpdateConfigPrefix << (src.update_config ? "1" : "0") << "\n";

  // Networks
  for (const auto& network : src.networks) {
    oss << "\n" << kStartNetworkBlock << "\n";

    oss << "\t" << kSsidPrefix << "\"" << network.ssid << "\"\n";
    oss << "\t" << kPriorityPrefix << network.priority << "\n";

    if (network.scan_ssid) {
      oss << "\t" << kScanSsidPrefix << "1\n";
    }

    oss << "\t" << kKeyMgmtPrefix << tokenFromEnum(network.key_mgmt) << "\n";
    switch (network.key_mgmt) {
      case KeyMgmt::NONE:
        break;
      case KeyMgmt::WPA_PSK:
        oss << "\t" << kPskPrefix << "\"" << network.psk << "\"\n";
        break;
      case KeyMgmt::SAE:
        oss << "\t" << kSaePasswordPrefix << "\"" << network.psk << "\"\n";
        oss << "\t" << kIeee80211wPrefix << 2 << "\n";  // PMF required.
        break;
      default:
        throw;
    }

    oss << kStopNetworkBlock << "\n";
  }

  return oss.str();
}

const char* Exporter::countryCodeToString(CountryCode cc)
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
}  // namespace wpa
}  // namespace tobas
