#include "../include/tobas_algorithm/crc.hpp"

namespace algo
{
CRC16Left::CRC16Left(uint16_t poly, uint16_t init_value, uint16_t out_xor)
  : poly_(poly), init_value_(init_value), out_xor_(out_xor)
{
  createTable();
}

uint16_t CRC16Left::compute(const uint8_t* buf, size_t len)
{
  uint16_t c = init_value_;
  for (size_t i = 0; i < len; ++i)
    c = (c << 8) ^ table_[((c >> 8) ^ buf[i]) & 0xFF];
  return c ^ out_xor_;
}

void CRC16Left::createTable()
{
  for (size_t i = 0; i < kTableSize; ++i)
  {
    uint16_t c = i << 8;
    for (int j = 0; j < 8; ++j)
      c = (c << 1) ^ ((c & 0x8000) ? poly_ : 0);
    table_[i] = c;
  }
}

CRC32Left::CRC32Left(uint32_t poly, uint32_t init_value, uint32_t out_xor)
  : poly_(poly), init_value_(init_value), out_xor_(out_xor)
{
  createTable();
}

uint32_t CRC32Left::compute(const uint8_t* buf, size_t len)
{
  uint32_t c = init_value_;
  for (size_t i = 0; i < len; ++i)
    c = (c << 8) ^ table_[((c >> 24) ^ buf[i]) & 0xFF];
  return c ^ out_xor_;
}

void CRC32Left::createTable()
{
  for (size_t i = 0; i < kTableSize; ++i)
  {
    uint32_t c = i << 24;
    for (int j = 0; j < 8; ++j)
      c = (c << 1) ^ ((c & 0x80000000) ? poly_ : 0);
    table_[i] = c;
  }
}
}  // namespace algo
