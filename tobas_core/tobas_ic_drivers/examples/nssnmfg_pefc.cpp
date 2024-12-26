#include <iostream>

#include <tobas_ic_drivers/nssnmfg_pefc.hpp>

using namespace std;

void onPacket(const driver::NssnmfgPEFC::Packet& packet)
{
  // Error
  const auto& error = packet.error;
  cout << boolalpha;
  cout << "Error:" << endl;
  cout << "\tStack Temperature Abnormal: " << error.stack_temperature_abnormal << endl;
  cout << "\tDCDC Current Too High: " << error.dcdc_current_too_high << endl;
  cout << "\tDCDC Voltage Too Low: " << error.dcdc_voltage_too_low << endl;
  cout << "\tFC Current Too High: " << error.fc_current_too_high << endl;
  cout << "\tFC Current Too Low: " << error.fc_current_too_low << endl;
  cout << "\tStack Leak: " << error.stack_leak << endl;
  cout << "\tBattery Voltage Abnormal: " << error.battery_voltage_abnormal << endl;
  cout << "\tHydrogen Level Too Low: " << error.hydrogen_level_too_low << endl;
  cout << noboolalpha;

  // Stack
  const auto& stack = packet.stack;
  cout << "Stack:" << endl;
  cout << "\tVoltage [mV]: " << stack.voltage << endl;
  cout << "\tCurrent [mA]: " << stack.current << endl;
  cout << "\tPower [W]: " << stack.power << endl;
  cout << "\tPressure [KPa]: " << stack.pressure << endl;
  cout << "\tTemperature [degC]: " << stack.temperature << endl;

  // Battery
  const auto& battery = packet.battery;
  cout << "Battery:" << endl;
  cout << "\tVoltage [mV]: " << battery.voltage << endl;
  cout << "\tCurrent [mA]: " << battery.current << endl;
  cout << "\tPower [W]: " << battery.power << endl;

  // Tank
  const auto& tank = packet.tank;
  cout << "Tank:" << endl;
  cout << "\tPressure [kPa]: " << tank.pressure << endl;

  cout << "----------" << endl;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    cerr << "Usage: " << argv[0] << " <Device>" << endl;
    return EXIT_FAILURE;
  }
  const auto device = argv[1];

  driver::NssnmfgPEFC pefc(&onPacket);

  if (!pefc.initialize(device))
  {
    cerr << "Failed to initialize fuel cell driver." << endl;
    return EXIT_FAILURE;
  }
  cout << "Fuel cell driver is initialized." << endl;

  pefc.start();
  pefc.spin();

  return EXIT_SUCCESS;
}
