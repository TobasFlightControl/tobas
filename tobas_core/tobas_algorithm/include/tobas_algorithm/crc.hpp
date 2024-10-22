#pragma once

#include <cinttypes>
#include <cstddef>

/* cf: [Cyclic Redundancy Check (CRC) を理解する | Qiita](https://qiita.com/tobira-code/items/dbcffc41f54201130b6c) */
namespace algo
{
class CRC
{
public:
  void initialize();

protected:
  static constexpr size_t kTableSize = 1 << 8;

  virtual void createTable() = 0;
};

class CRC16 : public CRC
{
public:
  /**
   * @brief Construct a new CRC16 object
   *
   * @param poly 生成多項式 (最高次数は省略)
   * @param init_value 初期値
   * @param out_xor 除算後のXOR
   */
  explicit CRC16(uint16_t poly, uint16_t init_value = 0, uint16_t out_xor = 0);

  virtual uint16_t compute(const uint8_t* buf, size_t len) = 0;

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
   * @param poly 生成多項式 (最高次数は省略)
   * @param init_value 初期値
   * @param out_xor 除算後のXOR
   */
  explicit CRC32(uint32_t poly, uint32_t init_value = 0, uint32_t out_xor = 0);

  virtual uint32_t compute(const uint8_t* buf, size_t len) = 0;

protected:
  const uint32_t poly_;
  const uint32_t init_value_;
  const uint32_t out_xor_;

  uint32_t table_[kTableSize];

  virtual void createTable() = 0;
};

class CRC16Right : public CRC16
{
public:
  using CRC16::CRC16;

  uint16_t compute(const uint8_t* buf, size_t len) override;

protected:
  void createTable() override;
};

class CRC16Left : public CRC16
{
public:
  using CRC16::CRC16;

  uint16_t compute(const uint8_t* buf, size_t len) override;

protected:
  void createTable() override;
};

class CRC32Right : public CRC32
{
public:
  using CRC32::CRC32;

  uint32_t compute(const uint8_t* buf, size_t len) override;

protected:
  void createTable() override;
};

class CRC32Left : public CRC32
{
public:
  using CRC32::CRC32;

  uint32_t compute(const uint8_t* buf, size_t len) override;

protected:
  void createTable() override;
};
}  // namespace algo
