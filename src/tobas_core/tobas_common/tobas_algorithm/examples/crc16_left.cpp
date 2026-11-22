// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <cstdint>
#include <ios>
#include <iostream>

#include <tobas_algorithm/crc.hpp>

int main()
{
  const uint16_t poly = (1 << 12) | (1 << 5) | (1 << 0);
  tobas::algo::CRC16Left crc(poly, 0xFFFF, 0xFFFF);
  crc.initialize();

  const uint8_t data[] = "0123456789";
  std::cout << "CRC-16: " << std::hex << std::uppercase << crc.compute(data, sizeof(data) - 1) << '\n';
}
