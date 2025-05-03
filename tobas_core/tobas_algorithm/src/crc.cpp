#include "../include/tobas_algorithm/crc.hpp"

namespace algo
{
void CRC::initialize()
{
  createTable();
}

CRC16::CRC16(uint16_t poly, uint16_t init_value, uint16_t out_xor)
  : poly_(poly), init_value_(init_value), out_xor_(out_xor)
{
}

CRC32::CRC32(uint32_t poly, uint32_t init_value, uint32_t out_xor)
  : poly_(poly), init_value_(init_value), out_xor_(out_xor)
{
}

uint16_t CRC16Right::compute(const uint8_t* buf, size_t len) const
{
  uint16_t c = init_value_;
  for (size_t i = 0; i < len; ++i) {
    c = table_[(c ^ buf[i]) & 0xFF] ^ (c >> 8);
  }
  return c ^ out_xor_;
}

void CRC16Right::createTable()
{
  for (size_t i = 0; i < kTableSize; ++i) {
    uint16_t c = i;
    for (size_t _ = 0; _ < 8; ++_) {
      c = (c & 1) ? (poly_ ^ (c >> 1)) : (c >> 1);
    }
    table_[i] = c;
  }
}

uint32_t CRC32Right::compute(const uint8_t* buf, size_t len) const
{
  uint32_t c = init_value_;
  for (size_t i = 0; i < len; ++i) {
    c = table_[(c ^ buf[i]) & 0xFF] ^ (c >> 8);
  }
  return c ^ out_xor_;
}

void CRC32Right::createTable()
{
  for (size_t i = 0; i < kTableSize; ++i) {
    uint32_t c = i;
    for (size_t _ = 0; _ < 8; ++_) {
      c = (c & 1) ? (poly_ ^ (c >> 1)) : (c >> 1);
    }
    table_[i] = c;
  }
}

uint16_t CRC16Left::compute(const uint8_t* buf, size_t len) const
{
  uint16_t c = init_value_ ^ 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    c = (c << 8) ^ table_[((c >> 8) ^ buf[i]) & 0xFF];
  }
  return c ^ out_xor_;
}

void CRC16Left::createTable()
{
  for (size_t i = 0; i < kTableSize; ++i) {
    uint16_t c = i << 8;
    for (size_t _ = 0; _ < 8; ++_) {
      c = (c << 1) ^ ((c & 0x8000) ? poly_ : 0);
    }
    table_[i] = c;
  }
}

uint32_t CRC32Left::compute(const uint8_t* buf, size_t len) const
{
  uint32_t c = init_value_ ^ 0xFFFFFFFF;
  for (size_t i = 0; i < len; ++i) {
    c = (c << 8) ^ table_[((c >> 24) ^ buf[i]) & 0xFF];
  }
  return c ^ out_xor_;
}

void CRC32Left::createTable()
{
  for (size_t i = 0; i < kTableSize; ++i) {
    uint32_t c = i << 24;
    for (size_t _ = 0; _ < 8; ++_) {
      c = (c << 1) ^ ((c & 0x80000000) ? poly_ : 0);
    }
    table_[i] = c;
  }
}
}  // namespace algo
