// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_transport_spi.hpp"

#include <cstring>

namespace tobas
{
namespace ublox
{
UbxTransportSpi::UbxTransportSpi(const char* _device) : device_(_device)
{
}

bool UbxTransportSpi::initialize() noexcept
{
  return spi_.initialize(device_, tx_buf_, rx_buf_, kSpiClockFreq);
}

std::optional<uint8_t> UbxTransportSpi::receiveByte() noexcept
{
  constexpr uint8_t kIdleData = 0xFF;  // Sent while there is no queued data.

  // SPI is full duplex, so send one byte of the queued data in exchange for each received byte.
  std::lock_guard<std::mutex> lock(send_queue_mutex_);
  tx_buf_[0] = send_queue_.empty() ? kIdleData : send_queue_.front();
  if (!spi_.transfer(1)) {
    return std::nullopt;
  }
  if (!send_queue_.empty()) {
    send_queue_.pop_front();
  }

  return rx_buf_[0];
}

bool UbxTransportSpi::send(const uint8_t* _data, size_t _length) noexcept
{
  if (_length > kSpiBufSize) {
    return false;
  }

  std::memcpy(tx_buf_, _data, _length);
  return spi_.transfer(_length);
}

bool UbxTransportSpi::enqueue(const uint8_t* _data, size_t _length) noexcept
{
  // Discard the queued data if it is not sent in time so that new data is not delayed.
  constexpr size_t kMaxSendQueueSize = 4096;

  std::lock_guard<std::mutex> lock(send_queue_mutex_);
  if (send_queue_.size() > kMaxSendQueueSize) {
    send_queue_.clear();
  }
  send_queue_.insert(send_queue_.end(), _data, _data + _length);
  return true;
}

bool UbxTransportSpi::hasPendingData() const noexcept
{
  std::lock_guard<std::mutex> lock(send_queue_mutex_);
  return !send_queue_.empty();
}
}  // namespace ublox
}  // namespace tobas
