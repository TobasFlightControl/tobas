#include <iostream>
#include <chrono>
#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/fstream.hpp>
#include <dh_eigen_tools/core.hpp>

#include <tobas_tools/constants.hpp>

#include "../../include/tobas_real/calibration/measure_sensor_noise.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;
using namespace chrono;
using namespace Eigen;

namespace tobas_real
{
MeasureSensorNoise::MeasureSensorNoise()
{
  // ルート権限を確認
  if (getuid())
  {
    throw runtime_error("Not root.");
  }

  // IMUドライバをセットアップ
  imu_.initialize();
  if (!imu_.probe())
  {
    throw runtime_error("IMU not enabled.");
  }

  // 気圧センサドライバをセットアップ
  barometer_.initialize();
  if (!barometer_.testConnection())
  {
    throw runtime_error("Barometer test failed.");
  }

  // PWMドライバをセットアップ
  for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
  {
    setupRCOutput(pwm_, channel);
  }
}

void MeasureSensorNoise::run()
{
  // 一定時間Disarmコマンドを送信
  cout << "Send disarm command for " << kDisarmDuration << " seconds." << endl;
  sendDisarm();
  cout << "Disarming finished. The motors are ready to rotate." << endl;

  // センサデータを取得
  Matrix<float, kDataCount, 3> acc_data;
  Matrix<float, kDataCount, 3> gyro_data;
  Matrix<float, kDataCount, 3> mag_data;
  Matrix<float, kDataCount, 1> pres_data;

  const auto start_time = system_clock::now();
  for (uint32_t i = 0; i < kDataCount; ++i)
  {
    // センサ情報を更新
    imu_.update();
    barometer_.refreshPressure();

    // モータが動いている状態でのノイズを計測するため，Armコマンドを送信
    for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.set_duty_cycle(channel, kPwmArm))
      {
        throw runtime_error("Failed to set PWM duty cycle.");
      }
    }

    // センサ情報を読み取る
    imu_.read_accelerometer(&acc_data(i, 0), &acc_data(i, 1), &acc_data(i, 2));
    imu_.read_accelerometer(&gyro_data(i, 0), &gyro_data(i, 1), &gyro_data(i, 2));
    imu_.read_magnetometer(&mag_data(i, 0), &mag_data(i, 1), &mag_data(i, 2));

    barometer_.readPressure();
    barometer_.calculatePressureAndTemperature();
    pres_data(i) = barometer_.getPressure() * 100;  // mbar -> Pa

    // Sleep
    usleep(kSleepTime);
  }
  const auto end_time = system_clock::now();

  // サンプリング周波数を計算
  const auto elapsed_time = duration_cast<microseconds>(end_time - start_time).count();
  const auto sample_freq = static_cast<float>(kDataCount * 1000000) / elapsed_time;
  cout << "Sampling frequency: " << sample_freq << " Hz" << endl;

  // 差分をとることで定常部分とランダムウォークを除去．分散2倍の白色ノイズのみが残る．
  constexpr uint32_t num = kDataCount - 1;
  const Matrix<float, num, 3> acc_diff = acc_data.bottomRows(num) - acc_data.topRows(num);
  const Matrix<float, num, 3> gyro_diff = gyro_data.bottomRows(num) - gyro_data.topRows(num);
  const Matrix<float, num, 3> mag_diff = mag_data.bottomRows(num) - mag_data.topRows(num);
  const Matrix<float, num, 1> pres_diff = pres_data.bottomRows(num) - pres_data.topRows(num);

  // 分散を計算．3軸のノイズ強度は等しいとして平均をとる．差分をとったため2で割る．
  const auto acc_var = eigen_tools::varianceCol(acc_diff).mean() / 2;
  const auto gyro_var = eigen_tools::varianceCol(gyro_diff).mean() / 2;
  const auto mag_var = eigen_tools::varianceCol(mag_diff).mean() / 2;
  const auto pres_var = eigen_tools::variance(pres_diff) / 2;

  // ノイズ密度を計算．白色ノイズを仮定し，全ての周波数帯域で同じ密度とする．
  const auto acc_noise_density = sqrt(acc_var / sample_freq);
  const auto gyro_noise_density = sqrt(gyro_var / sample_freq);
  const auto mag_noise_density = sqrt(mag_var / sample_freq);
  const auto pres_noise_density = sqrt(pres_var / sample_freq);
  cout << "Accel noise density: " << acc_noise_density << " m/s^2/sqrt(Hz)" << endl;
  cout << "Gyro noise density: " << gyro_noise_density << " rad/s/sqrt(Hz)" << endl;
  cout << "Compass noise density: " << mag_noise_density << " /sqrt(Hz)" << endl;
  cout << "Pressure noise density: " << pres_noise_density << " Pa/sqrt(Hz)" << endl;

  // Configに保存
  boost::property_tree::ptree pt;
  if (dh_std::fileExists(kConfigPath))
  {
    boost::property_tree::ini_parser::read_ini(kConfigPath, pt);
  }
  pt.put(kConfigKey_AccNoiseDensity, acc_noise_density);
  pt.put(kConfigKey_GyroNoiseDensity, gyro_noise_density);
  pt.put(kConfigKey_MagNoiseDensity, mag_noise_density);
  pt.put(kConfigKey_PressureNoiseDensity, pres_noise_density);
  boost::property_tree::ini_parser::write_ini(kConfigPath, pt);
  cout << "The result is saved to '" << kConfigPath << "'." << endl;
}

void MeasureSensorNoise::sendDisarm()
{
  const auto start_time = system_clock::now();
  while (duration_cast<milliseconds>(system_clock::now() - start_time).count()
         < kDisarmDuration * 1000)
  {
    for (uint32_t channel = 0; channel < kServoRailSize; ++channel)
    {
      if (!pwm_.set_duty_cycle(channel, kPwmDisarm))
      {
        throw runtime_error("Failed to set PWM duty cycle.");
      }
    }
    usleep(kDisarmInterval * 1000000);
  }
}
}  // namespace tobas_real
