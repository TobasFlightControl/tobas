#pragma once

/**
 * @brief 標準大気
 * cf. https://pigeon-poppo.com/standard-atmosphere/
 */
namespace tobas_std
{
/**
 * @brief ジオポテンシャル高度から幾何高度を求める．
 *
 * @param gph ジオポテンシャル高度 [m]
 * @return double 幾何高度 [m]
 */
double gphToAltitude(const double& gph);

/**
 * @brief 幾何高度からジオポテンシャル高度を求める．
 *
 * @param altitude 幾何高度 [m]
 * @return double ジオポテンシャル高度 [m]
 */
double altitudeToGPH(const double& altitude);

/**
 * @brief ジオポテンシャル高度から標準大気の温度を求める．
 *
 * @param gph ジオポテンシャル高度 [m]
 * @return double 標準大気の温度 [K]
 */
double gphToTemperature(const double& gph);

/**
 * @brief 幾何高度から標準大気の温度を求める．
 *
 * @param altitude 幾何高度 [m]
 * @return double 標準大気の温度 [K]
 */
double altitudeToTemperature(const double& altitude);

/**
 * @brief 大気圧から標準大気の温度を求める．
 *
 * @param p 大気圧 [Pa]
 * @return double 標準大気の温度 [K]
 *
 * @note 対流圏を想定．
 */
double pressureToTemperature(const double& p);

/**
 * @brief ジオポテンシャル高度から標準大気の圧力を求める．
 *
 * @param gph ジオポテンシャル高度 [m]
 * @return double 標準大気の圧力 [Pa]
 */
double gphToPressure(const double& gph);

/**
 * @brief 幾何高度から標準大気の圧力を求める．
 *
 * @param altitude 幾何高度 [m]
 * @return double 標準大気の圧力 [Pa]
 */
double altitudeToPressure(const double& altitude);

/**
 * @brief 大気温度から標準大気の圧力を求める．
 *
 * @param T 大気温度 [K]
 * @return double 標準大気の圧力 [Pa]
 *
 * @note 対流圏を想定．
 */
double temperatureToPressure(const double& T);

/**
 * @brief ジオポテンシャル高度から標準大気の密度を求める．
 *
 * @param gph ジオポテンシャル高度 [m]
 * @return double 標準大気の密度 [kg/m^3]
 */
double gphToDensity(const double& gph);

/**
 * @brief 幾何高度から標準大気の密度を求める．
 *
 * @param altitude 幾何高度 [m]
 * @return double 標準大気の密度 [kg/m^3]
 */
double altitudeToDensity(const double& altitude);

/**
 * @brief 大気圧から標準大気の密度を求める．
 *
 * @param p 大気圧 [Pa]
 * @return double 標準大気の密度 [kg/m^3]
 */
double pressureToDensity(const double& p);

/**
 * @brief 対流圏の標準大気を仮定し，大気圧[Pa]から幾何高度[m]を求める．
 * cf. file:///home/dohi/Downloads/IPSJ-DICOMO2013164.pdf
 *
 * @param pressure 気圧 [Pa]
 * @return double 幾何高度 [m]
 */
double pressureToAltitude(const double& pressure);

/**
 * @brief 対流圏の標準大気を仮定し，大気圧[Pa]から幾何高度[m]を求める．加えて分散も変換する．
 * cf. file:///home/dohi/Downloads/IPSJ-DICOMO2013164.pdf
 *
 * @param pressure 気圧 [Pa]
 * @param pressure_var 気圧の分散 [Pa^2]
 * @param altitude 幾何高度 [m] (出力)
 * @param altitude_var 幾何高度の分散 [m^2] (出力)
 */
void pressureToAltitude(
  const double& pressure,
  const double& pressure_var,
  double& altitude,
  double& altitude_var);
}  // namespace tobas_std
