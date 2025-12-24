#include <cmath>
#include <iostream>

#include <tobas_fc1xx_core/battery.hpp>
#include <tobas_fc1xx_core/iis2mdc.hpp>
#include <tobas_fc1xx_core/ilps22qs.hpp>
#include <tobas_ic_drivers/stmicro/ism330dlc.hpp>
#include <tobas_ic_drivers/ublox/zed_f9p.hpp>
#include <tobas_math/linalg.hpp>
#include <tobas_sbus_driver/sbus.hpp>
#include <tobas_std_tools/ansi_text_styles.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_time_tools/rate.hpp>

#define SAMPLING_PERIOD 100ms
#define SAMPLING_COUNT 50

using namespace std;
using namespace std::chrono_literals;
namespace ch = std::chrono;

bool testImu()
{
  stm::ISM330DLC imu;

  if (!imu.initialize("/dev/spidev0.0")) {
    cerr << "Failed to initialize IMU." << endl;
    return false;
  }

  if (!imu.setAccelOutputDataRate(stm::ISM330DLC::odr_xl_t::ODR_XL_6664HZ)) {
    cerr << "Failed to set accelerometer output data rate." << endl;
    return false;
  }
  if (!imu.setGyroOutputDataRate(stm::ISM330DLC::odr_g_t::ODR_G_6664HZ)) {
    cerr << "Failed to set gyroscope output data rate." << endl;
    return false;
  }

  this_thread::sleep_for(100ms);

  double ax, ay, az, gx, gy, gz;
  tim::Rate rate(SAMPLING_PERIOD);

  for (int i = 0; i < SAMPLING_COUNT; ++i) {
    if (!imu.readImu(ax, ay, az, gx, gy, gz)) {
      cerr << "Failed to read IMU." << endl;
      return false;
    }

    cout << "Accel [m/s^2]: " << ax << ", " << ay << ", " << az << endl;
    cout << "Gyro [rad/s] : " << gx << ", " << gy << ", " << gz << endl;

    if (math::norm(ax, ay, az - tbs::kGravity) > 1.) {
      cerr << "Abnormal accel detected." << endl;
      return false;
    }
    if (math::norm(gx, gy, gz) > 0.3) {
      cerr << "Abnormal gyro detected." << endl;
      return false;
    }

    rate.sleep();
  }

  return true;
}

bool testMagnetometer()
{
  fc1xx::IIS2MDC mag;

  if (!mag.initialize()) {
    cerr << "Failed to initialize magnetometer." << endl;
    return false;
  }

  this_thread::sleep_for(100ms);

  double mx, my, mz;
  tim::Rate rate(SAMPLING_PERIOD);

  for (int i = 0; i < SAMPLING_COUNT; ++i) {
    if (!mag.readMag(mx, my, mz)) {
      cerr << "Failed to read magnetic field." << endl;
      return false;
    }

    cout << "Magnetic Field [gauss]: " << mx << ", " << my << ", " << mz << endl;

    if (math::norm(mx, my, mz) > 1.5) {  // 標準の地磁気の大きさ (0.5くらい) の3倍まで許容
      cerr << "Abnormal magnetic field detected." << endl;
      return false;
    }

    rate.sleep();
  }

  return true;
}

bool testBarometer()
{
  fc1xx::ILPS22QS baro;

  if (!baro.initialize()) {
    cerr << "Failed to initialize barometer." << endl;
    return false;
  }

  this_thread::sleep_for(100ms);

  double pres, temp;
  tim::Rate rate(SAMPLING_PERIOD);

  for (int i = 0; i < SAMPLING_COUNT; ++i) {
    if (!baro.readPressure(pres)) {
      cerr << "Failed to read pressure." << endl;
      return false;
    }
    if (!baro.readTemperature(temp)) {
      cerr << "Failed to read temperature." << endl;
      return false;
    }

    const auto pres_hpa = pres / 100.;

    cout << "Pressure [hPa]     : " << pres_hpa << endl;
    cout << "Temperature [degC]: " << temp << endl;

    if (pres_hpa < 900. || 1100. < pres_hpa) {
      cerr << "Abnormal air pressure detected." << endl;
      return false;
    }
    if (temp < 0. || 80. < temp) {
      cerr << "Abnormal temperature detected." << endl;
      return false;
    }

    rate.sleep();
  }

  return true;
}

bool testPowerSensor()
{
  fc1xx::Battery batt;

  if (!batt.initialize()) {
    cerr << "Failed to initialize ADC." << endl;
    return EXIT_FAILURE;
  }

  this_thread::sleep_for(100ms);

  float volt, curr;
  tim::Rate rate(SAMPLING_PERIOD);

  for (int i = 0; i < SAMPLING_COUNT; ++i) {
    if (!batt.read(volt, curr)) {
      cerr << "Failed to read battery status." << endl;
      return EXIT_FAILURE;
    }

    cout << "Voltage [V]: " << volt << endl;
    cout << "Current [A]: " << curr << endl;

    if (volt < 5. || 50. < volt) {
      cerr << "Abnormal voltage detected." << endl;
      return false;
    }
    if (curr <= 0. || 10. < curr) {
      cerr << "Abnormal current detected." << endl;
      return false;
    }

    rate.sleep();
  }

  return true;
}

bool testGnssReceiver()
{
  ublox::ZEDF9P gnss;

  if (!gnss.initialize("/dev/spidev1.2")) {
    cerr << "Failed to initialize GNSS driver." << endl;
    return false;
  }

  if (!gnss.configureMeasurementRate(1000)) {
    cerr << "Failed to configure measurement rate." << endl;
    return false;
  }

  // Enable GNSS
  if (!gnss.enableGps()) {
    cerr << "Failed to enable GPS." << endl;
    return false;
  }
  if (!gnss.enableSbas()) {
    cerr << "Failed to enable SBAS." << endl;
    return false;
  }
  if (!gnss.enableQzss()) {
    cerr << "Failed to enable QZSS." << endl;
    return false;
  }

  // Enable messages
  if (!gnss.enableMsg(ublox::ZEDF9P::CLASS_NAV, ublox::ZEDF9P::NAV_PVT, true)) {
    cerr << "Failed to enable NAV_PVT message." << endl;
    return false;
  }

  const auto start_time = ch::steady_clock::now();

  while (ch::steady_clock::now() - start_time < 3s) {
    if (!gnss.update(false)) {
      cerr << "Failed to update GNSS driver." << endl;
      return false;
    }

    if (gnss.latestClass() != ublox::ZEDF9P::CLASS_NAV) {
      continue;
    }

    switch (gnss.latestId()) {
      case ublox::ZEDF9P::NAV_PVT:
        return true;
      default:
        continue;
    }
  }

  cerr << "Timeout before receiving the first GPS message." << endl;
  return false;
}

bool testSbus()
{
  bool sbus_received = false;

  tobas::SBUS sbus([&sbus_received](const tobas::SBUS::Packet&) { sbus_received = true; });

  if (!sbus.initialize("/dev/ttyAMA0")) {
    cerr << "Failed to initialize S.BUS driver." << endl;
    return false;
  }

  sbus.start();

  const auto start_time = ch::steady_clock::now();
  while (ch::steady_clock::now() - start_time < 1min) {
    if (sbus_received) {
      return true;
    }
    this_thread::sleep_for(10ms);
  }

  cerr << "Timeout before the first S.BUS package." << endl;
  return false;
}

int main()
{
  // IMU
  cout << "Testing IMU..." << endl;
  if (!testImu()) {
    cerr << "IMU test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "IMU test passed." << endl;

  // Magnetometer
  cout << "Testing magnetometer..." << endl;
  if (!testMagnetometer()) {
    cerr << "Magnetometer test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "Magnetometer test passed." << endl;

  // Barometer
  cout << "Testing baro..." << endl;
  if (!testBarometer()) {
    cerr << "Barometer test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "Barometer test passed." << endl;

  // Power sensor
  cout << "Testing power sensor..." << endl;
  if (!testPowerSensor()) {
    cerr << "Power sensor test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "Power sensor test passed." << endl;

  // GNSS Receiver
  cout << "Testing GNSS receiver..." << endl;
  if (!testGnssReceiver()) {
    cerr << "GNSS receiver test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "GNSS receiver test passed." << endl;

  // S.BUS
  cout << "Testing S.BUS..." << endl;
  if (!testSbus()) {
    cerr << "S.BUS test failed." << endl;
    return EXIT_FAILURE;
  }
  cout << "S.BUS test passed." << endl;

  cout << GREEN_PREFIX << "All tests passed." << COLOR_RESET << endl;
  return EXIT_SUCCESS;
}
