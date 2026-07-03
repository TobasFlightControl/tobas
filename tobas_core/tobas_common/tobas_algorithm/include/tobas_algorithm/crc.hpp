// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>
#include <cstddef>

/* cf: https://qiita.com/tobira-code/items/dbcffc41f54201130b6c */
namespace tobas
{
namespace algo
{
class CRC
{
public:
  void initialize() noexcept;

protected:
  static constexpr size_t kTableSize = 1 << 8;

  virtual void createTable() noexcept = 0;
};

class CRC16 : public CRC
{
public:
  /**
   * @brief Construct a new CRC16 object
   *
   * @param poly Generator polynomial, omitting the highest-order term.
   * @param init_value Initial value.
   * @param out_xor XOR after division.
   */
  explicit CRC16(uint16_t poly, uint16_t init_value = 0, uint16_t out_xor = 0) noexcept;

  virtual uint16_t compute(const uint8_t* buf, size_t len) const noexcept = 0;

protected:
  const uint16_t poly_;
  const uint16_t init_value_;
  const uint16_t out_xor_;

  uint16_t table_[kTableSize];
};

class CRC32 : public CRC
{
public:
  /**
   * @brief Construct a new CRC32 object
   *
   * @param poly Generator polynomial, omitting the highest-order term.
   * @param init_value Initial value.
   * @param out_xor XOR after division.
   */
  explicit CRC32(uint32_t poly, uint32_t init_value = 0, uint32_t out_xor = 0) noexcept;

  virtual uint32_t compute(const uint8_t* buf, size_t len) const noexcept = 0;

protected:
  const uint32_t poly_;
  const uint32_t init_value_;
  const uint32_t out_xor_;

  uint32_t table_[kTableSize];

  virtual void createTable() noexcept = 0;
};

class CRC16Right : public CRC16
{
public:
  enum Polynomial : uint16_t
  {
    CRC_16_CCITT = 0x8408,
    CRC_16_IBM = 0xA001,
  };

  using CRC16::CRC16;

  uint16_t compute(const uint8_t* buf, size_t len) const noexcept override;

protected:
  void createTable() noexcept override;
};

class CRC16Left : public CRC16
{
public:
  enum Polynomial : uint16_t
  {
    CRC_16_CCITT = 0x1021,
    CRC_16_IBM = 0x8005,
  };

  using CRC16::CRC16;

  uint16_t compute(const uint8_t* buf, size_t len) const noexcept override;

protected:
  void createTable() noexcept override;
};

class CRC32Right : public CRC32
{
public:
  enum Polynomial : uint32_t
  {
    CRC_32 = 0xEDB88320,
    CRC_32C = 0x82F63B78,
    CRC_32K = 0xEB31D82E,
  };

  using CRC32::CRC32;

  uint32_t compute(const uint8_t* buf, size_t len) const noexcept override;

protected:
  void createTable() noexcept override;
};

class CRC32Left : public CRC32
{
public:
  enum Polynomial : uint32_t
  {
    CRC_32 = 0x04C11DB7,
    CRC_32C = 0x1EDC6F41,
    CRC_32K = 0x741B8CD7,
  };

  using CRC32::CRC32;

  uint32_t compute(const uint8_t* buf, size_t len) const noexcept override;

protected:
  void createTable() noexcept override;
};
}  // namespace algo
}  // namespace tobas
