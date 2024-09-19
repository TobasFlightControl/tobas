#pragma once

/* Country Codes: https://countrycode.org/ */
namespace wpa
{
enum country_code_t
{
  AF,
  AL,
  DZ,
  AS,
  AD,
  AO,
  AI,
  AQ,
  AG,
  AR,
  AM,
  AW,
  AU,
  AT,
  AZ,
  BS,
  BH,
  BD,
  BB,
  BY,
  BE,
  BZ,
  BJ,
  BM,
  BT,
  BO,
  BA,
  BW,
  BR,
  IO,
  VG,
  BN,
  BG,
  BF,
  BI,
  KH,
  CM,
  CA,
  CV,
  KY,
  CF,
  TD,
  CL,
  CN,
  CX,
  CC,
  CO,
  KM,
  CK,
  CR,
  HR,
  CU,
  CW,
  CY,
  CZ,
  CD,
  DK,
  DJ,
  DM,
  DO,
  TL,
  EC,
  EG,
  SV,
  GQ,
  ER,
  EE,
  ET,
  FK,
  FO,
  FJ,
  FI,
  FR,
  PF,
  GA,
  GM,
  GE,
  DE,
  GH,
  GI,
  GR,
  GL,
  GD,
  GU,
  GT,
  GG,
  GN,
  GW,
  GY,
  HT,
  HN,
  HK,
  HU,
  IS,
  IN,
  ID,
  IR,
  IQ,
  IE,
  IM,
  IL,
  IT,
  CI,
  JM,
  JP,
  JE,
  JO,
  KZ,
  KE,
  KI,
  XK,
  KW,
  KG,
  LA,
  LV,
  LB,
  LS,
  LR,
  LY,
  LI,
  LT,
  LU,
  MO,
  MK,
  MG,
  MW,
  MY,
  MV,
  ML,
  MT,
  MH,
  MR,
  MU,
  YT,
  MX,
  FM,
  MD,
  MC,
  MN,
  ME,
  MS,
  MA,
  MZ,
  MM,
  NA,
  NR,
  NP,
  NL,
  AN,
  NC,
  NZ,
  NI,
  NE,
  NG,
  NU,
  KP,
  MP,
  NO,
  OM,
  PK,
  PW,
  PS,
  PA,
  PG,
  PY,
  PE,
  PH,
  PN,
  PL,
  PT,
  PR,
  QA,
  CG,
  RE,
  RO,
  RU,
  RW,
  BL,
  SH,
  KN,
  LC,
  MF,
  PM,
  VC,
  WS,
  SM,
  ST,
  SA,
  SN,
  RS,
  SC,
  SL,
  SG,
  SX,
  SK,
  SI,
  SB,
  SO,
  ZA,
  KR,
  SS,
  ES,
  LK,
  SD,
  SR,
  SJ,
  SZ,
  SE,
  CH,
  SY,
  TW,
  TJ,
  TZ,
  TH,
  TG,
  TK,
  TO,
  TT,
  TN,
  TR,
  TM,
  TC,
  TV,
  VI,
  UG,
  UA,
  AE,
  GB,
  US,
  UY,
  UZ,
  VU,
  VA,
  VE,
  VN,
  WF,
  EH,
  YE,
  ZM,
  ZW,
};

namespace country_code
{
static constexpr char Afghanistan[] = "AF";
static constexpr char Albania[] = "AL";
static constexpr char Algeria[] = "DZ";
static constexpr char American_Samoa[] = "AS";
static constexpr char Andorra[] = "AD";
static constexpr char Angola[] = "AO";
static constexpr char Anguilla[] = "AI";
static constexpr char Antarctica[] = "AQ";
static constexpr char Antigua_and_Barbuda[] = "AG";
static constexpr char Argentina[] = "AR";
static constexpr char Armenia[] = "AM";
static constexpr char Aruba[] = "AW";
static constexpr char Australia[] = "AU";
static constexpr char Austria[] = "AT";
static constexpr char Azerbaijan[] = "AZ";
static constexpr char Bahamas[] = "BS";
static constexpr char Bahrain[] = "BH";
static constexpr char Bangladesh[] = "BD";
static constexpr char Barbados[] = "BB";
static constexpr char Belarus[] = "BY";
static constexpr char Belgium[] = "BE";
static constexpr char Belize[] = "BZ";
static constexpr char Benin[] = "BJ";
static constexpr char Bermuda[] = "BM";
static constexpr char Bhutan[] = "BT";
static constexpr char Bolivia[] = "BO";
static constexpr char Bosnia_and_Herzegovina[] = "BA";
static constexpr char Botswana[] = "BW";
static constexpr char Brazil[] = "BR";
static constexpr char British_Indian_OceanTerritory[] = "IO";
static constexpr char British_Virgin_Islands[] = "VG";
static constexpr char Brunei[] = "BN";
static constexpr char Bulgaria[] = "BG";
static constexpr char Burkina_Faso[] = "BF";
static constexpr char Burundi[] = "BI";
static constexpr char Cambodia[] = "KH";
static constexpr char Cameroon[] = "CM";
static constexpr char Canad[] = "CA";
static constexpr char Cape_Verde[] = "CV";
static constexpr char Cayman_Islands[] = "KY";
static constexpr char Central_African_Republic[] = "CF";
static constexpr char Chad[] = "TD";
static constexpr char Chile[] = "CL";
static constexpr char China[] = "CN";
static constexpr char Christmas_Island[] = "CX";
static constexpr char Cocos_Islands[] = "CC";
static constexpr char Colombia[] = "CO";
static constexpr char Comoros[] = "KM";
static constexpr char Cook_Islands[] = "CK";
static constexpr char Costa_Rica[] = "CR";
static constexpr char Croatia[] = "HR";
static constexpr char Cuba[] = "CU";
static constexpr char Curacao[] = "CW";
static constexpr char Cyprus[] = "CY";
static constexpr char Czech_Republic[] = "CZ";
static constexpr char Democratic_Republic_of_the_Congo[] = "CD";
static constexpr char Denmark[] = "DK";
static constexpr char Djibouti[] = "DJ";
static constexpr char Dominica[] = "DM";
static constexpr char Dominican_Republic[] = "DO";
static constexpr char East_Timor[] = "TL";
static constexpr char Ecuador[] = "EC";
static constexpr char Egypt[] = "EG";
static constexpr char El_Salvador[] = "SV";
static constexpr char Equatorial_Guinea[] = "GQ";
static constexpr char Eritrea[] = "ER";
static constexpr char Estonia[] = "EE";
static constexpr char Ethiopia[] = "ET";
static constexpr char Falkland_Islands[] = "FK";
static constexpr char Faroe_Islands[] = "FO";
static constexpr char Fiji[] = "FJ";
static constexpr char Finland[] = "FI";
static constexpr char France[] = "FR";
static constexpr char French_Polynesia[] = "PF";
static constexpr char Gabon[] = "GA";
static constexpr char Gambia[] = "GM";
static constexpr char Georgia[] = "GE";
static constexpr char Germany[] = "DE";
static constexpr char Ghana[] = "GH";
static constexpr char Gibraltar[] = "GI";
static constexpr char Greece[] = "GR";
static constexpr char Greenland[] = "GL";
static constexpr char Grenada[] = "GD";
static constexpr char Guam[] = "GU";
static constexpr char Guatemala[] = "GT";
static constexpr char Guernsey[] = "GG";
static constexpr char Guinea[] = "GN";
static constexpr char Guinea_Bissau[] = "GW";
static constexpr char Guyana[] = "GY";
static constexpr char Haiti[] = "HT";
static constexpr char Honduras[] = "HN";
static constexpr char Hong_Kong[] = "HK";
static constexpr char Hungary[] = "HU";
static constexpr char Iceland[] = "IS";
static constexpr char India[] = "IN";
static constexpr char Indonesia[] = "ID";
static constexpr char Iran[] = "IR";
static constexpr char Iraq[] = "IQ";
static constexpr char Ireland[] = "IE";
static constexpr char Isle_of_Man[] = "IM";
static constexpr char Israel[] = "IL";
static constexpr char Italy[] = "IT";
static constexpr char Ivory_Coast[] = "CI";
static constexpr char Jamaica[] = "JM";
static constexpr char Japan[] = "JP";
static constexpr char Jersey[] = "JE";
static constexpr char Jordan[] = "JO";
static constexpr char Kazakhsta[] = "KZ";
static constexpr char Kenya[] = "KE";
static constexpr char Kiribati[] = "KI";
static constexpr char Kosovo[] = "XK";
static constexpr char Kuwait[] = "KW";
static constexpr char Kyrgyzstan[] = "KG";
static constexpr char Laos[] = "LA";
static constexpr char Latvia[] = "LV";
static constexpr char Lebanon[] = "LB";
static constexpr char Lesotho[] = "LS";
static constexpr char Liberia[] = "LR";
static constexpr char Libya[] = "LY";
static constexpr char Liechtenstein[] = "LI";
static constexpr char Lithuania[] = "LT";
static constexpr char Luxembourg[] = "LU";
static constexpr char Macau[] = "MO";
static constexpr char Macedonia[] = "MK";
static constexpr char Madagascar[] = "MG";
static constexpr char Malawi[] = "MW";
static constexpr char Malaysia[] = "MY";
static constexpr char Maldives[] = "MV";
static constexpr char Mali[] = "ML";
static constexpr char Malta[] = "MT";
static constexpr char Marshall_Islands[] = "MH";
static constexpr char Mauritania[] = "MR";
static constexpr char Mauritius[] = "MU";
static constexpr char Mayotte[] = "YT";
static constexpr char Mexico[] = "MX";
static constexpr char Micronesia[] = "FM";
static constexpr char Moldova[] = "MD";
static constexpr char Monaco[] = "MC";
static constexpr char Mongolia[] = "MN";
static constexpr char Montenegro[] = "ME";
static constexpr char Montserrat[] = "MS";
static constexpr char Morocco[] = "MA";
static constexpr char Mozambique[] = "MZ";
static constexpr char Myanmar[] = "MM";
static constexpr char Namibia[] = "NA";
static constexpr char Nauru[] = "NR";
static constexpr char Nepal[] = "NP";
static constexpr char Netherlands[] = "NL";
static constexpr char Netherlands_Antilles[] = "AN";
static constexpr char New_Caledonia[] = "NC";
static constexpr char New_Zealand[] = "NZ";
static constexpr char Nicaragua[] = "NI";
static constexpr char Niger[] = "NE";
static constexpr char Nigeria[] = "NG";
static constexpr char Niue[] = "NU";
static constexpr char North_Korea[] = "KP";
static constexpr char Northern_Mariana_Islands[] = "MP";
static constexpr char Norway[] = "NO";
static constexpr char Oman[] = "OM";
static constexpr char Pakistan[] = "PK";
static constexpr char Palau[] = "PW";
static constexpr char Palestine[] = "PS";
static constexpr char Panama[] = "PA";
static constexpr char Papua_New_Guinea[] = "PG";
static constexpr char Paraguay[] = "PY";
static constexpr char Peru[] = "PE";
static constexpr char Philippines[] = "PH";
static constexpr char Pitcairn[] = "PN";
static constexpr char Poland[] = "PL";
static constexpr char Portugal[] = "PT";
static constexpr char Puerto_Rico[] = "PR";
static constexpr char Qatar[] = "QA";
static constexpr char Republic_of_the_Congo[] = "CG";
static constexpr char Reunion[] = "RE";
static constexpr char Romania[] = "RO";
static constexpr char Russi[] = "RU";
static constexpr char Rwanda[] = "RW";
static constexpr char Saint_Barthelemy[] = "BL";
static constexpr char Saint_Helena[] = "SH";
static constexpr char Saint_Kitts_and_Nevis[] = "KN";
static constexpr char Saint_Lucia[] = "LC";
static constexpr char Saint_Martin[] = "MF";
static constexpr char Saint_Pierre_and_Miquelon[] = "PM";
static constexpr char Saint_Vincent_and_the_Grenadines[] = "VC";
static constexpr char Samoa[] = "WS";
static constexpr char San_Marino[] = "SM";
static constexpr char Sao_Tome_and_Principe[] = "ST";
static constexpr char Saudi_Arabia[] = "SA";
static constexpr char Senegal[] = "SN";
static constexpr char Serbia[] = "RS";
static constexpr char Seychelles[] = "SC";
static constexpr char Sierra_Leone[] = "SL";
static constexpr char Singapore[] = "SG";
static constexpr char Sint_Maarten[] = "SX";
static constexpr char Slovakia[] = "SK";
static constexpr char Slovenia[] = "SI";
static constexpr char Solomon_Islands[] = "SB";
static constexpr char Somalia[] = "SO";
static constexpr char South_Africa[] = "ZA";
static constexpr char South_Korea[] = "KR";
static constexpr char South_Sudan[] = "SS";
static constexpr char Spain[] = "ES";
static constexpr char Sri_Lanka[] = "LK";
static constexpr char Sudan[] = "SD";
static constexpr char Suriname[] = "SR";
static constexpr char Svalbard_and_Jan_Mayen[] = "SJ";
static constexpr char Swaziland[] = "SZ";
static constexpr char Sweden[] = "SE";
static constexpr char Switzerland[] = "CH";
static constexpr char Syria[] = "SY";
static constexpr char Taiwan[] = "TW";
static constexpr char Tajikistan[] = "TJ";
static constexpr char Tanzania[] = "TZ";
static constexpr char Thailand[] = "TH";
static constexpr char Togo[] = "TG";
static constexpr char Tokelau[] = "TK";
static constexpr char Tonga[] = "TO";
static constexpr char Trinidad_and_Tobago[] = "TT";
static constexpr char Tunisia[] = "TN";
static constexpr char Turkey[] = "TR";
static constexpr char Turkmenistan[] = "TM";
static constexpr char Turks_and_Caicos_Islands[] = "TC";
static constexpr char Tuvalu[] = "TV";
static constexpr char US_Virgin_Islands[] = "VI";
static constexpr char Uganda[] = "UG";
static constexpr char Ukraine[] = "UA";
static constexpr char United_Arab_Emirates[] = "AE";
static constexpr char United_Kingdom[] = "GB";
static constexpr char United_State[] = "US";
static constexpr char Uruguay[] = "UY";
static constexpr char Uzbekistan[] = "UZ";
static constexpr char Vanuatu[] = "VU";
static constexpr char Vatican[] = "VA";
static constexpr char Venezuela[] = "VE";
static constexpr char Vietnam[] = "VN";
static constexpr char Wallis_and_Futuna[] = "WF";
static constexpr char Western_Sahara[] = "EH";
static constexpr char Yemen[] = "YE";
static constexpr char Zambia[] = "ZM";
static constexpr char Zimbabwe[] = "ZW";
}  // namespace country_code
}  // namespace wpa
