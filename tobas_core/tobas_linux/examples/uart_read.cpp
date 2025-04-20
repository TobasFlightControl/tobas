#include <iostream>

#include <cxxopts.hpp>

#include <tobas_linux/uart_dev.hpp>

#define DEVICE "device"
#define BAUDRATE "baudrate"
#define DATA_BITS "data_bits"
#define STOP_BITS "Stop_bits"
#define PARITY "parity"
#define HUNGUP "hungup"
#define HELP "help"

using namespace std;

int main(int argc, char** argv)
{
  cxxopts::Options options("uart_read", "Simple UART Reader");
  auto opts = options.add_options();
  opts(HELP, "Print usage");
  opts(DEVICE, "UART device", cxxopts::value<string>()->default_value("/dev/ttyUSB0"));
  opts(BAUDRATE, "Baudrate", cxxopts::value<int>()->default_value("9600"));
  opts(DATA_BITS, "The number of data bits", cxxopts::value<int>()->default_value("8"));
  opts(STOP_BITS, "The number of stop bits", cxxopts::value<int>()->default_value("0"));
  opts(PARITY, "Enable parity check", cxxopts::value<string>()->default_value("none"));
  opts(HUNGUP, "Enable hungup close");

  cxxopts::ParseResult result;
  try
  {
    result = options.parse(argc, argv);
  }
  catch (const cxxopts::exceptions::exception& e)
  {
    cerr << "Failed to parse arguments: " << e.what() << endl;
    return EXIT_FAILURE;
  }

  if (result.count(HELP))
  {
    std::cout << options.help() << std::endl;
    return EXIT_SUCCESS;
  }

  const auto device = result[DEVICE].as<string>();
  const auto baudrate = result[BAUDRATE].as<int>();
  const auto data_bits = result[DATA_BITS].as<int>();
  const auto stop_bits = result[STOP_BITS].as<int>();
  const auto parity = result[PARITY].as<string>();
  const auto hungup = result.count(HUNGUP) > 0;

  cout << "----------------------------------------" << endl;
  cout << "UART Configurations:" << endl;
  cout << "\tDevice: " << device << endl;
  cout << "\tBaudrate: " << baudrate << endl;
  cout << "\tData bits: " << data_bits << endl;
  cout << "\tStop bits: " << stop_bits << endl;
  cout << "\tParity: " << parity << endl;
  cout << "\tHungup close: " << boolalpha << hungup << noboolalpha << endl;
  cout << "----------------------------------------" << endl;

  linux::UARTdev uart;

  if (!uart.initialize(device.c_str()))
    return EXIT_FAILURE;

  if (!uart.setBaudRate(baudrate))
    return EXIT_FAILURE;

  if (!uart.setDataBits(data_bits))
    return EXIT_FAILURE;

  switch (stop_bits)
  {
    case 0:
      break;
    case 1:
      if (!uart.setSingleStopBit())
        return EXIT_FAILURE;
      break;
    case 2:
      if (!uart.setDoubleStopBit())
        return EXIT_FAILURE;
      break;
    default:
      cerr << "The number of stop bits is invalid: " << stop_bits << endl;
      return EXIT_FAILURE;
  }

  if (parity == "none")
  {
    if (!uart.disableParity())
      return EXIT_FAILURE;
  }
  else if (parity == "odd")
  {
    if (!uart.enableParity(linux::UARTdev::parity_mode_t::PARITY_ODD))
      return EXIT_FAILURE;
  }
  else if (parity == "even")
  {
    if (!uart.enableParity(linux::UARTdev::parity_mode_t::PARITY_EVEN))
      return EXIT_FAILURE;
  }

  if (hungup)
  {
    if (!uart.enableHungupClose())
      return EXIT_FAILURE;
  }
  else
  {
    if (!uart.disableHungupClose())
      return EXIT_FAILURE;
  }

  uint8_t data;
  cout << hex;

  while (true)
  {
    if (!uart.receive(&data, 1))
      continue;
    cout << data << " ";
  }
}
