#include <cassert>

#include "../include/tobas_std_tools/geometry.hpp"
#include "../include/tobas_std_tools/math.hpp"

namespace tobas_std
{
void eulerToQuaternion(
  const double& roll,
  const double& pitch,
  const double& yaw,
  double& x,
  double& y,
  double& z,
  double& w)
{
  const double cx = cos(0.5 * roll);
  const double sx = sin(0.5 * roll);
  const double cy = cos(0.5 * pitch);
  const double sy = sin(0.5 * pitch);
  const double cz = cos(0.5 * yaw);
  const double sz = sin(0.5 * yaw);

  x = sx * cy * cz - cx * sy * sz;
  y = sx * cy * sz + cx * sy * cz;
  z = -sx * sy * cz + cx * cy * sz;
  w = sx * sy * sz + cx * cy * cz;
}

void quaternionToEuler(
  const double& x,
  const double& y,
  const double& z,
  const double& w,
  double& roll,
  double& pitch,
  double& yaw)
{
  assert(isClose(sqr(x) + sqr(y) + sqr(z) + sqr(w), 1.));

  const double sy = -2 * (x * z - y * w);
  const bool unlocked = !isClose(fabs(sy), 1.);

  roll = unlocked ? atan2(2 * (y * z + x * w), 2 * (sqr(w) + sqr(z)) - 1) : 0.;
  pitch = asin(sy);
  yaw = unlocked ? atan2(2 * (x * y + z * w), 2 * (sqr(w) + sqr(x)) - 1) :
                   atan2(-2 * (x * y - z * w), 2 * (sqr(w) + sqr(y)) - 1);
}

void imuToEuler(
  const double& ax,
  const double& ay,
  const double& az,
  const double& mx,
  const double& my,
  const double& mz,
  const double& mx_ref,
  const double& my_ref,
  const double&,  // mz_ref
  double& roll,
  double& pitch,
  double& yaw)
{
  roll = atan2(ay, az);
  pitch = atan2(ax, sqrt(sqr(ay) + sqr(az)));

  const double x = mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll);
  const double y = my * cos(roll) - mz * sin(roll);
  yaw = atan2(my_ref * x - mx_ref * y, mx_ref * x + my_ref * y);
}

void imuToQuaternion(
  const double& ax,
  const double& ay,
  const double& az,
  const double& mx,
  const double& my,
  const double& mz,
  const double& mx_ref,
  const double& my_ref,
  const double& mz_ref,
  double& qx,
  double& qy,
  double& qz,
  double& qw)
{
  double roll, pitch, yaw;
  imuToEuler(ax, ay, az, mx, my, mz, mx_ref, my_ref, mz_ref, roll, pitch, yaw);
  eulerToQuaternion(roll, pitch, yaw, qx, qy, qz, qw);
}

void gpsToCartAbsolute(
  const double& latitude,
  const double& longitude,
  const double& altitude,
  double& x,
  double& y,
  double& z)
{
  constexpr double long_radius = 6378137.;          // 長半径 [m]
  constexpr double eccentricity = 0.0818191908426;  // 離心率 [-]

  const double phi = deg2rad(latitude);
  const double lam = deg2rad(longitude);

  const double cos_lat = cos(phi);
  const double sin_lat = sin(phi);
  const double cos_lon = cos(lam);
  const double sin_lon = sin(lam);

  const double N = long_radius / sqrt(1. - sqr(eccentricity * sin(phi)));
  x = (N + altitude) * cos_lat * cos_lon;
  y = (N + altitude) * cos_lat * sin_lon;
  z = (N * (1. - sqr(eccentricity)) + altitude) * sin_lat;
}

void gpsToCartRelative(
  const double& latitude,
  const double& longitude,
  const double& latitude_0,
  const double& longitude_0,
  double& x,
  double& y)
{
  // 緯度経度・平面直角座標系原点をラジアンに直す
  const double phi = deg2rad(latitude);
  const double lam = deg2rad(longitude);
  const double phi_0 = deg2rad(latitude_0);
  const double lam_0 = deg2rad(longitude_0);

  // 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
  constexpr double m0 = 0.9999;
  constexpr double a = 6378137.;
  constexpr double F = 298.257222101;

  // (1) n,A_i,alpha_iの計算
  constexpr double n = 1. / (2 * F - 1);
  constexpr double n2 = n * n;
  constexpr double n3 = n2 * n;
  constexpr double n4 = n3 * n;
  constexpr double n5 = n4 * n;

  constexpr double A0 = 1 + n2 / 4. + n4 / 64.;
  constexpr double A1 = -(3. / 2) * (n - n3 / 8. - n5 / 64.);
  constexpr double A2 = (15. / 16) * (n2 - n4 / 4.);
  constexpr double A3 = -(35. / 48) * (n3 - (5. / 16) * n5);
  constexpr double A4 = (315. / 512) * n4;
  constexpr double A5 = -(693. / 1280) * n5;

  constexpr double a1 = (1. / 2) * n - (2. / 3) * n2 + (5. / 16) * n3 + (41. / 180) * n4 - (127. / 288) * n5;
  constexpr double a2 = (13. / 48) * n2 - (3. / 5) * n3 + (557. / 1440) * n4 + (281. / 630) * n5;
  constexpr double a3 = (61. / 240) * n3 - (103. / 140) * n4 + (15061. / 26880) * n5;
  constexpr double a4 = (49561. / 161280) * n4 - (179. / 168) * n5;
  constexpr double a5 = (34729. / 80640) * n5;

  // (2) A,Sの計算
  constexpr double m_ = (m0 * a) / (1. + n);
  constexpr double A_ = m_ * A0;  // [m]
  const double S_ =
    m_
    * (A0 * phi_0 + A1 * sin(2 * phi_0) + A2 * sin(4 * phi_0) + A3 * sin(6 * phi_0) + A4 * sin(8 * phi_0) + A5 * sin(10 * phi_0));

  // (3) lam_c,lam_sの計算
  const double lam_c = cos(lam - lam_0);
  const double lam_s = sin(lam - lam_0);

  // (4) t,t_の計算
  const double t = sinh(atanh(sin(phi)) - ((2 * sqrt(n)) / (1 + n)) * atanh(((2 * sqrt(n)) / (1 + n)) * sin(phi)));
  const double t_ = sqrt(1 + t * t);

  // (5) xi',eta'の計算
  const double xi2 = atan(t / lam_c);  // [rad]
  const double eta2 = atanh(lam_s / t_);

  // (6) x,yの計算
  x =
    A_
      * (xi2 + a1 * sin(2 * xi2) * cosh(2 * eta2) + a2 * sin(4 * xi2) * cosh(4 * eta2) + a3 * sin(6 * xi2) * cosh(6 * eta2) + a4 * sin(8 * xi2) * cosh(8 * eta2) + a5 * sin(10 * xi2) * cosh(10 * eta2))
    - S_;  // [m]
  y =
    A_
    * (eta2 + a1 * cos(2 * xi2) * sinh(2 * eta2) + a2 * cos(4 * xi2) * sinh(4 * eta2) + a3 * cos(6 * xi2) * sinh(6 * eta2) + a4 * cos(8 * xi2) * sinh(8 * eta2) + a5 * cos(10 * xi2) * sinh(10 * eta2));  // [m]

  // このままだと東が正になっているので反転する
  y *= -1;
}

void cartToGpsRelative(
  const double& x,
  const double& y,
  const double& latitude_0,
  const double& longitude_0,
  double& latitude,
  double& longitude)
{
  // 平面直角座標系原点をラジアンに直す
  const double phi_0 = deg2rad(latitude_0);
  const double lam_0 = deg2rad(longitude_0);

  // 定数 (a, F: 世界測地系-測地基準系1980（GRS80）楕円体)
  constexpr double a = 6378137.;
  constexpr double m0 = 0.9999;
  constexpr double F = 298.257222101;

  // (1) n, A_i, beta_i, delta_iの計算
  constexpr double n = 1. / (2 * F - 1);
  constexpr double n2 = n * n;
  constexpr double n3 = n2 * n;
  constexpr double n4 = n3 * n;
  constexpr double n5 = n4 * n;
  constexpr double n6 = n5 * n;

  constexpr double A0 = 1 + n2 / 4. + n4 / 64.;
  constexpr double A1 = -(3. / 2) * (n - n3 / 8. - n5 / 64.);
  constexpr double A2 = (15. / 16) * (n2 - n4 / 4.);
  constexpr double A3 = -(35. / 48) * (n3 - (5. / 16) * n5);
  constexpr double A4 = (315. / 512) * n4;
  constexpr double A5 = -(693. / 1280) * n5;

  constexpr double b1 = (1. / 2) * n - (2. / 3) * n2 + (37. / 96) * n3 - (1. / 360) * n4 - (81. / 512) * n5;
  constexpr double b2 = (1. / 48) * n2 + (1. / 15) * n3 - (437. / 1440) * n4 + (46. / 105) * n5;
  constexpr double b3 = (17. / 480) * n3 - (37. / 840) * n4 - (209. / 4480) * n5;
  constexpr double b4 = (4397. / 161280) * n4 - (11. / 504) * n5;
  constexpr double b5 = (4583. / 161280) * n5;

  constexpr double d1 = 2. * n - (2. / 3) * n2 - 2. * n3 + (116. / 45) * n4 + (26. / 45) * n5 - (2854. / 675) * n6;
  constexpr double d2 = (7. / 3) * n2 - (8. / 5) * n3 - (227. / 45) * n4 + (2704. / 315) * n5 + (2323. / 945) * n6;
  constexpr double d3 = (56. / 15) * n3 - (136. / 35) * n4 - (1262. / 105) * n5 + (73814. / 2835) * n6;
  constexpr double d4 = (4279. / 630) * n4 - (332. / 35) * n5 - (399572. / 14175) * n6;
  constexpr double d5 = (4174. / 315) * n5 - (144838. / 6237) * n6;
  constexpr double d6 = (601676. / 22275) * n6;

  // (2) A,Sの計算
  constexpr double m_ = (m0 * a) / (1. + n);
  constexpr double A_ = m_ * A0;  // [m]
  const double S_ =
    m_
    * (A0 * phi_0 + A1 * sin(2 * phi_0) + A2 * sin(4 * phi_0) + A3 * sin(6 * phi_0) + A4 * sin(8 * phi_0) + A5 * sin(10 * phi_0));

  // (3) xi,etaの計算
  const double xi = (x + S_) / A_;
  const double eta = y / A_;

  // (4) xi',eta'の計算
  const double xi2 = xi - b1 * sin(2 * xi) * cosh(2 * eta) - b2 * sin(4 * xi) * cosh(4 * eta)
                     - b3 * sin(6 * xi) * cosh(6 * eta) - b4 * sin(8 * xi) * cosh(8 * eta)
                     - b5 * sin(10 * xi) * cosh(10 * eta);
  const double eta2 = eta - b1 * cos(2 * xi) * sinh(2 * eta) - b2 * cos(4 * xi) * sinh(4 * eta)
                      - b3 * cos(6 * xi) * sinh(6 * eta) - b4 * cos(8 * xi) * sinh(8 * eta)
                      - b5 * cos(10 * xi) * sinh(10 * eta);

  // (5) chiの計算
  const double chi = asin(sin(xi2) / cosh(eta2));  // [rad]

  // (6) 北緯，東経の計算
  const double latitude_rad = chi + d1 * sin(2 * chi) + d2 * sin(4 * chi) + d3 * sin(6 * chi) + d4 * sin(8 * chi)
                              + d5 * sin(10 * chi) + d6 * sin(12 * chi);  // [rad]
  const double longitude_rad = lam_0 - atan(sinh(eta2) / cos(xi2));       // [rad]

  // ラジアンを度になおす
  latitude = rad2deg(latitude_rad);
  longitude = rad2deg(longitude_rad);
}
}  // namespace tobas_std
