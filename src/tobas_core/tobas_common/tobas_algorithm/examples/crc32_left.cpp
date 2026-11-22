// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdint>
#include <ios>
#include <iostream>

#include <tobas_algorithm/crc.hpp>

int main()
{
  tobas::algo::CRC32Left crc(tobas::algo::CRC32Left::CRC_32, 0x00000000, 0x00000000);
  crc.initialize();

  const uint8_t data[] = "Example data for CRC calculation";
  std::cout << "CRC-32: " << std::hex << std::uppercase << crc.compute(data, sizeof(data) - 1) << '\n';
}
