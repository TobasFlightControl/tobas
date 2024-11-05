#pragma once

#include <cstdint>
#include <cstddef>
#include <linux/spi/spidev.h>

namespace linux
{
class SPIdev
{
  static constexpr size_t kBufSize = 256;

public:
  // 送受信データを格納するバッファ．
  // データがメモリ上で連続することが保証されているため，2バイト以上の型にキャストして使うことも可能．
  alignas(kBufSize) uint8_t tx[kBufSize] = { 0 };
  alignas(kBufSize) uint8_t rx[kBufSize] = { 0 };

  explicit SPIdev();
  ~SPIdev();

  bool initialize(const char* spi_dev, uint32_t speed_hz);
  bool transfer(uint32_t length);

private:
  spi_ioc_transfer spi_transfer_;
  int spi_fd_ = -1;
};
}  // namespace linux
