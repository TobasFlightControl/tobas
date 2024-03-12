#include <stdexcept>
#include <iostream>
#include <boost/property_tree/ini_parser.hpp>

#include <navio2/ADC.h>
#include <tobas_real/common.hpp>

using namespace std;

int main()
{
  // Get ADC coefficient
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(tobas_real::kConfigPath, pt);
  const double adc_coef = pt.get<double>(tobas_real::kConfigKey_AdcCoef);

  // Initialize ADC driver
  ADC adc;
  adc.initialize();

  while (true)
  {
    // Read from ADC converter
    const int a2_value = adc.read(tobas_real::kPowerModuleVoltageChannel);
    if (a2_value < 0)
      throw runtime_error("Failed to read battery voltage.");

    // Compute voltage
    const double voltage = static_cast<double>(a2_value) * adc_coef * 1e-3;
    cout << "Voltage [V]: " << voltage << endl;

    sleep(1);
  }
}
