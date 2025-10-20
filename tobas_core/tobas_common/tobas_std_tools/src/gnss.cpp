#include "tobas_std_tools/gnss.hpp"

#include <chrono>
#include <ctime>

#include <tobas_math/core.hpp>

#include "tobas_std_tools/unit_conversions.hpp"

using namespace std;

namespace tobas_std
{
long computeGPSDelayFromToW(uint32_t gps_tow_ms)
{
  // 現在のUTC時刻を取得
  const auto now = chrono::system_clock::now();
  const auto now_c = chrono::system_clock::to_time_t(now);
  const auto utc_time = gmtime(&now_c);

  // その週の日曜日0時0分0秒を計算
  utc_time->tm_sec = 0;
  utc_time->tm_min = 0;
  utc_time->tm_hour = 0;
  utc_time->tm_mday -= utc_time->tm_wday;  // 現在の曜日から日曜日に戻る
  const auto start_of_week = chrono::system_clock::from_time_t(mktime(utc_time));

  // 週のはじめから現在までの経過時間を計算
  const auto duration = now - start_of_week;
  const auto cur_tow_ms = chrono::duration_cast<chrono::milliseconds>(duration).count();

  // GPS TOWと現在のTOWの差を計算
  const auto diff_ms = cur_tow_ms - gps_tow_ms;

  return diff_ms;
}

void gnssToCartAbsolute(
  const double& latitude,
  const double& longitude,
  const double& altitude,
  double& x,
  double& y,
  double& z)
{
  constexpr double long_radius = 6378137.;          // 長半径 [m]
  constexpr double eccentricity = 0.0818191908426;  // 離心率 [-]

  const auto phi = deg2rad(latitude);
  const auto lam = deg2rad(longitude);

  const auto cos_lat = cos(phi);
  const auto sin_lat = sin(phi);
  const auto cos_lon = cos(lam);
  const auto sin_lon = sin(lam);

  const auto N = long_radius / sqrt(1 - math::sqr(eccentricity * sin(phi)));
  x = (N + altitude) * cos_lat * cos_lon;
  y = (N + altitude) * cos_lat * sin_lon;
  z = (N * (1 - math::sqr(eccentricity)) + altitude) * sin_lat;
}

void gnssToCartRelative(
  const double& latitude,
  const double& longitude,
  const double& latitude_0,
  const double& longitude_0,
  double& east,
  double& north)
{
  // 緯度経度・平面直角座標系原点をラジアンに直す
  const auto phi = deg2rad(latitude);
  const auto lam = deg2rad(longitude);
  const auto phi_0 = deg2rad(latitude_0);
  const auto lam_0 = deg2rad(longitude_0);

  // 定数 (a, F: 世界測地系-測地基準系1980 (GRS80) 楕円体)
  constexpr double m0 = 0.9999;
  constexpr double a = 6378137.;
  constexpr double F = 298.257222101;

  // (1) n,A_i,alpha_iの計算
  constexpr auto n = 1 / (2 * F - 1);
  constexpr auto n2 = n * n;
  constexpr auto n3 = n2 * n;
  constexpr auto n4 = n3 * n;
  constexpr auto n5 = n4 * n;

  constexpr auto A0 = 1 + n2 / 4 + n4 / 64;
  constexpr auto A1 = -(3. / 2) * (n - n3 / 8 - n5 / 64);
  constexpr auto A2 = (15. / 16) * (n2 - n4 / 4);
  constexpr auto A3 = -(35. / 48) * (n3 - (5. / 16) * n5);
  constexpr auto A4 = (315. / 512) * n4;
  constexpr auto A5 = -(693. / 1280) * n5;

  constexpr auto a1 = (1. / 2) * n - (2. / 3) * n2 + (5. / 16) * n3 + (41. / 180) * n4 - (127. / 288) * n5;
  constexpr auto a2 = (13. / 48) * n2 - (3. / 5) * n3 + (557. / 1440) * n4 + (281. / 630) * n5;
  constexpr auto a3 = (61. / 240) * n3 - (103. / 140) * n4 + (15061. / 26880) * n5;
  constexpr auto a4 = (49561. / 161280) * n4 - (179. / 168) * n5;
  constexpr auto a5 = (34729. / 80640) * n5;

  // (2) A,Sの計算
  constexpr auto m_ = (m0 * a) / (1 + n);
  constexpr auto A_ = m_ * A0;  // [m]
  const auto S_ = m_ * (A0 * phi_0 + A1 * sin(2 * phi_0) + A2 * sin(4 * phi_0) + A3 * sin(6 * phi_0) +
                        A4 * sin(8 * phi_0) + A5 * sin(10 * phi_0));

  // (3) lam_c,lam_sの計算
  const auto lam_c = cos(lam - lam_0);
  const auto lam_s = sin(lam - lam_0);

  // (4) t,t_の計算
  const auto t = sinh(atanh(sin(phi)) - ((2 * sqrt(n)) / (1 + n)) * atanh(((2 * sqrt(n)) / (1 + n)) * sin(phi)));
  const auto t_ = sqrt(1 + t * t);

  // (5) xi',eta'の計算
  const auto xi2 = atan(t / lam_c);  // [rad]
  const auto eta2 = atanh(lam_s / t_);

  // (6) x,yの計算
  const auto x = A_ * (xi2 + a1 * sin(2 * xi2) * cosh(2 * eta2) + a2 * sin(4 * xi2) * cosh(4 * eta2) +
                       a3 * sin(6 * xi2) * cosh(6 * eta2) + a4 * sin(8 * xi2) * cosh(8 * eta2) +
                       a5 * sin(10 * xi2) * cosh(10 * eta2)) -
                 S_;  // [m]
  const auto y = A_ * (eta2 + a1 * cos(2 * xi2) * sinh(2 * eta2) + a2 * cos(4 * xi2) * sinh(4 * eta2) +
                       a3 * cos(6 * xi2) * sinh(6 * eta2) + a4 * cos(8 * xi2) * sinh(8 * eta2) +
                       a5 * cos(10 * xi2) * sinh(10 * eta2));  // [m]

  east = y;
  north = x;
}

void cartToGnssRelative(
  const double& east,
  const double& north,
  const double& latitude_0,
  const double& longitude_0,
  double& latitude,
  double& longitude)
{
  // 平面直角座標系原点をラジアンに直す
  const auto phi_0 = deg2rad(latitude_0);
  const auto lam_0 = deg2rad(longitude_0);

  // 定数 (a, F: 世界測地系-測地基準系1980 (GRS80) 楕円体)
  constexpr double a = 6378137.;
  constexpr double m0 = 0.9999;
  constexpr double F = 298.257222101;

  // (1) n, A_i, beta_i, delta_iの計算
  constexpr auto n = 1 / (2 * F - 1);
  constexpr auto n2 = n * n;
  constexpr auto n3 = n2 * n;
  constexpr auto n4 = n3 * n;
  constexpr auto n5 = n4 * n;
  constexpr auto n6 = n5 * n;

  constexpr auto A0 = 1 + n2 / 4 + n4 / 64;
  constexpr auto A1 = -(3. / 2) * (n - n3 / 8 - n5 / 64);
  constexpr auto A2 = (15. / 16) * (n2 - n4 / 4);
  constexpr auto A3 = -(35. / 48) * (n3 - (5. / 16) * n5);
  constexpr auto A4 = (315. / 512) * n4;
  constexpr auto A5 = -(693. / 1280) * n5;

  constexpr auto b1 = (1. / 2) * n - (2. / 3) * n2 + (37. / 96) * n3 - (1. / 360) * n4 - (81. / 512) * n5;
  constexpr auto b2 = (1. / 48) * n2 + (1. / 15) * n3 - (437. / 1440) * n4 + (46. / 105) * n5;
  constexpr auto b3 = (17. / 480) * n3 - (37. / 840) * n4 - (209. / 4480) * n5;
  constexpr auto b4 = (4397. / 161280) * n4 - (11. / 504) * n5;
  constexpr auto b5 = (4583. / 161280) * n5;

  constexpr auto d1 = 2 * n - (2. / 3) * n2 - 2 * n3 + (116. / 45) * n4 + (26. / 45) * n5 - (2854. / 675) * n6;
  constexpr auto d2 = (7. / 3) * n2 - (8. / 5) * n3 - (227. / 45) * n4 + (2704. / 315) * n5 + (2323. / 945) * n6;
  constexpr auto d3 = (56. / 15) * n3 - (136. / 35) * n4 - (1262. / 105) * n5 + (73814. / 2835) * n6;
  constexpr auto d4 = (4279. / 630) * n4 - (332. / 35) * n5 - (399572. / 14175) * n6;
  constexpr auto d5 = (4174. / 315) * n5 - (144838. / 6237) * n6;
  constexpr auto d6 = (601676. / 22275) * n6;

  // (2) A,Sの計算
  constexpr auto m_ = (m0 * a) / (1 + n);
  constexpr auto A_ = m_ * A0;  // [m]
  const auto S_ = m_ * (A0 * phi_0 + A1 * sin(2 * phi_0) + A2 * sin(4 * phi_0) + A3 * sin(6 * phi_0) +
                        A4 * sin(8 * phi_0) + A5 * sin(10 * phi_0));

  // (3) xi,etaの計算
  const auto xi = (north + S_) / A_;
  const auto eta = east / A_;

  // (4) xi',eta'の計算
  const auto xi2 = xi - b1 * sin(2 * xi) * cosh(2 * eta) - b2 * sin(4 * xi) * cosh(4 * eta) -
                   b3 * sin(6 * xi) * cosh(6 * eta) - b4 * sin(8 * xi) * cosh(8 * eta) -
                   b5 * sin(10 * xi) * cosh(10 * eta);
  const auto eta2 = eta - b1 * cos(2 * xi) * sinh(2 * eta) - b2 * cos(4 * xi) * sinh(4 * eta) -
                    b3 * cos(6 * xi) * sinh(6 * eta) - b4 * cos(8 * xi) * sinh(8 * eta) -
                    b5 * cos(10 * xi) * sinh(10 * eta);

  // (5) chiの計算
  const auto chi = asin(sin(xi2) / cosh(eta2));  // [rad]

  // (6) 北緯，東経の計算
  const auto latitude_rad = chi + d1 * sin(2 * chi) + d2 * sin(4 * chi) + d3 * sin(6 * chi) + d4 * sin(8 * chi) +
                            d5 * sin(10 * chi) + d6 * sin(12 * chi);  // [rad]
  const auto longitude_rad = lam_0 + atan(sinh(eta2) / cos(xi2));     // [rad]

  // ラジアンを度になおす
  latitude = rad2deg(latitude_rad);
  longitude = rad2deg(longitude_rad);
}
}  // namespace tobas_std
