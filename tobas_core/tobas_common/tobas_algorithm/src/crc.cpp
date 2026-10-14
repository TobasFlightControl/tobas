// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_algorithm/crc.hpp"

namespace tobas::algo
{
void CRC::initialize() noexcept
{
  createTable();
}

CRC16::CRC16(uint16_t _poly, uint16_t _init_value, uint16_t _out_xor) noexcept
  : poly_(_poly), init_value_(_init_value), out_xor_(_out_xor)
{
}

CRC32::CRC32(uint32_t _poly, uint32_t _init_value, uint32_t _out_xor) noexcept
  : poly_(_poly), init_value_(_init_value), out_xor_(_out_xor)
{
}

uint16_t CRC16Right::compute(const uint8_t* _buf, size_t _len) const noexcept
{
  uint16_t c = init_value_;
  for (size_t i = 0; i < _len; ++i) {
    c = table_[(c ^ _buf[i]) & 0xFF] ^ (c >> 8);
  }
  return c ^ out_xor_;
}

void CRC16Right::createTable() noexcept
{
  for (size_t i = 0; i < kTableSize; ++i) {
    uint16_t c = i;
    for (size_t _ = 0; _ < 8; ++_) {
      c = (c & 1) ? (poly_ ^ (c >> 1)) : (c >> 1);
    }
    table_[i] = c;
  }
}

uint32_t CRC32Right::compute(const uint8_t* _buf, size_t _len) const noexcept
{
  uint32_t c = init_value_;
  for (size_t i = 0; i < _len; ++i) {
    c = table_[(c ^ _buf[i]) & 0xFF] ^ (c >> 8);
  }
  return c ^ out_xor_;
}

void CRC32Right::createTable() noexcept
{
  for (size_t i = 0; i < kTableSize; ++i) {
    uint32_t c = i;
    for (size_t _ = 0; _ < 8; ++_) {
      c = (c & 1) ? (poly_ ^ (c >> 1)) : (c >> 1);
    }
    table_[i] = c;
  }
}

uint16_t CRC16Left::compute(const uint8_t* _buf, size_t _len) const noexcept
{
  uint16_t c = init_value_ ^ 0xFFFF;
  for (size_t i = 0; i < _len; ++i) {
    c = (c << 8) ^ table_[((c >> 8) ^ _buf[i]) & 0xFF];
  }
  return c ^ out_xor_;
}

void CRC16Left::createTable() noexcept
{
  for (size_t i = 0; i < kTableSize; ++i) {
    uint16_t c = i << 8;
    for (size_t _ = 0; _ < 8; ++_) {
      c = (c << 1) ^ ((c & 0x8000) ? poly_ : 0);
    }
    table_[i] = c;
  }
}

uint32_t CRC32Left::compute(const uint8_t* _buf, size_t _len) const noexcept
{
  uint32_t c = init_value_ ^ 0xFFFFFFFF;
  for (size_t i = 0; i < _len; ++i) {
    c = (c << 8) ^ table_[((c >> 24) ^ _buf[i]) & 0xFF];
  }
  return c ^ out_xor_;
}

void CRC32Left::createTable() noexcept
{
  for (size_t i = 0; i < kTableSize; ++i) {
    uint32_t c = i << 24;
    for (size_t _ = 0; _ < 8; ++_) {
      c = (c << 1) ^ ((c & 0x80000000) ? poly_ : 0);
    }
    table_[i] = c;
  }
}
}  // namespace tobas::algo
