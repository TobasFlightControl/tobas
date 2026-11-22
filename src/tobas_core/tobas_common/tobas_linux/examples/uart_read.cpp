// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <ios>
#include <iostream>
#include <ostream>
#include <string>

#include <cxxopts.hpp>

#include <tobas_linux/uart_dev.hpp>

int main(int _argc, char* _argv[])
{
  // Command-line option names.
  constexpr char kDevice[] = "device";
  constexpr char kBaudRate[] = "baudrate";
  constexpr char kDataBits[] = "data_bits";
  constexpr char kStopBits[] = "stop_bits";
  constexpr char kParity[] = "parity";
  constexpr char kHungUp[] = "hungup";
  constexpr char kHelp[] = "help";

  cxxopts::Options options("uart_read", "Simple UART Reader");
  auto opts = options.add_options();
  opts(kHelp, "Print usage");
  opts(kDevice, "UART device", cxxopts::value<std::string>()->default_value("/dev/ttyUSB0"));
  opts(kBaudRate, "Baudrate", cxxopts::value<int>()->default_value("9600"));
  opts(kDataBits, "The number of data bits {5, 6, 7, 8}", cxxopts::value<int>()->default_value("8"));
  opts(kStopBits, "The number of stop bits {0, 1, 2}", cxxopts::value<int>()->default_value("0"));
  opts(kParity, "Parity check mode {none, odd, even}", cxxopts::value<std::string>()->default_value("none"));
  opts(kHungUp, "Enable hungup close", cxxopts::value<bool>()->default_value("false"));

  cxxopts::ParseResult result;
  try {
    result = options.parse(_argc, _argv);
  }
  catch (const cxxopts::exceptions::exception& e) {
    std::cerr << "Failed to parse arguments: " << e.what() << '\n';
    return EXIT_FAILURE;
  }

  if (result.count(kHelp)) {
    std::cout << options.help() << '\n';
    return EXIT_SUCCESS;
  }

  const auto device = result[kDevice].as<std::string>();
  const auto baudrate = result[kBaudRate].as<int>();
  const auto data_bits = result[kDataBits].as<int>();
  const auto stop_bits = result[kStopBits].as<int>();
  const auto parity = result[kParity].as<std::string>();
  const auto hungup = result[kHungUp].as<bool>();

  std::cout << "----------------------------------------\n";
  std::cout << "UART Configurations:\n";
  std::cout << "\tDevice: " << device << '\n';
  std::cout << "\tBaudrate: " << baudrate << '\n';
  std::cout << "\tData bits: " << data_bits << '\n';
  std::cout << "\tStop bits: " << stop_bits << '\n';
  std::cout << "\tParity: " << parity << '\n';
  std::cout << "\tHungup close: " << std::boolalpha << hungup << std::noboolalpha << '\n';
  std::cout << "----------------------------------------\n";

  tobas::linux::UARTdev uart;

  if (!uart.initialize(device.c_str(), true)) {
    return EXIT_FAILURE;
  }

  if (!uart.setBaudRate(baudrate)) {
    return EXIT_FAILURE;
  }

  if (!uart.setDataBits(data_bits)) {
    return EXIT_FAILURE;
  }

  switch (stop_bits) {
    case 0:
      break;
    case 1:
      if (!uart.setSingleStopBit()) {
        return EXIT_FAILURE;
      }
      break;
    case 2:
      if (!uart.setDoubleStopBit()) {
        return EXIT_FAILURE;
      }
      break;
    default:
      std::cerr << "The number of stop bits is invalid: " << stop_bits << '\n';
      return EXIT_FAILURE;
  }

  if (parity == "none") {
    if (!uart.disableParity()) {
      return EXIT_FAILURE;
    }
  }
  else if (parity == "odd") {
    if (!uart.enableParity(tobas::linux::UARTdev::ParityMode::kOdd)) {
      return EXIT_FAILURE;
    }
  }
  else if (parity == "even") {
    if (!uart.enableParity(tobas::linux::UARTdev::ParityMode::kEven)) {
      return EXIT_FAILURE;
    }
  }
  else {
    std::cerr << "Invalid parity mode: " << parity << '\n';
    return EXIT_FAILURE;
  }

  if (hungup) {
    if (!uart.enableHungupClose()) {
      return EXIT_FAILURE;
    }
  }
  else {
    if (!uart.disableHungupClose()) {
      return EXIT_FAILURE;
    }
  }

  uint8_t data;
  while (true) {
    if (!uart.receive(&data, 1)) {
      continue;
    }
    std::cout << std::setw(2) << std::setfill('0') << std::hex << std::uppercase << static_cast<int>(data) << ' '
              << std::flush;
  }
}
