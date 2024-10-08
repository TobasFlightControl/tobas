#pragma once

#include <cinttypes>
#include <cstddef>

namespace algo
{
/* cf: https://qiita.com/tobira-code/items/dbcffc41f54201130b6c */
class CRC32Left
{
  static constexpr size_t kTableSize = 1 << 8;

public:
  explicit CRC32Left(uint32_t poly, uint32_t init_value = 0, uint32_t out_xor = 0);

  uint32_t compute(uint8_t* buf, size_t len);

private:
  const uint32_t poly_;
  const uint32_t init_value_;
  const uint32_t out_xor_;

  uint32_t table_[kTableSize];

  void createTable();
};
}  // namespace algo
