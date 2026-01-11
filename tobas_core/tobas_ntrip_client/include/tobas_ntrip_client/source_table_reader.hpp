#pragma once

#include <cmath>
#include <string>
#include <vector>

namespace ntrip
{
class SourceTableReader
{
  // access http://rtk2go.com:2101/ to get the example source table
public:
  // source tableの文字列データを読む．
  explicit SourceTableReader(const std::string& data);
  // 各mount_pointに対する与えられた緯度・経度までの距離を計算し，近い順に並べて，moint pointの名前をその順で返す．緯度・経度はdegree．
  std::vector<std::string> sortMountPoints(const double& latitude, const double& longitude);

private:
  static constexpr double kDEG_TO_RAD = M_PI / 180.0;
  enum Index : u_int8_t
  {
    kTYPE = 0,          // e.g. STR
    kMOUNT_POINT,       // マウントポイント名 e.g. NEAR-JPNc
    kIDENTIFIER,        // 発信元ID, 地域名等 e.g. Japan Central
    kFORMAT,            // データフォーマット e.g. RTCM 3.2
    kFORMAT_DETAILS,    // フォーマット詳細 e.g. 1002(1),1006(10)
    kCARRIER,           // 搬送波位相情報を含むか．0 = 含まない，1 = L1を含む，2 = L1&L2を含む
    kNAV_SYSTEM,        // GNSS名
    kNETWORK,           // ネットワーク名
    kCOUNTRY,           // ISO3166で定義されている国コード e.g. JPN
    kLATITUDE,          // 北緯 e.g. 0.0
    kLONGITUDE,         // 東経 e.g. 0.0
    kNMEA,              // クライアントがNMEAで自位置を送信可能か 0 = 不可，1 = 可
    kSOLUTION,          // 単一基準点かネットワーク型基準点か 0 = 単一，1 = ネットワーク
    kGENERATOR,         // データ作成ソフト e.g. sNTRIP
    kCOMPR_ENCRYP,      // 圧縮/暗号アルゴリズム
    kAUTHENTIFICATION,  // 接続認証 N = 認証なし，B = Basic認証，D = Digest認証
    kFEE,               // 課金 N = なし，Y = あり
    kBITRATE,           // ビットレート(bps)
    kMISC,              // その他
  };

  // source tableから読み取ったmoint pointのデータ
  std::vector<std::vector<std::string>> moint_points_;
};
}  // namespace ntrip
