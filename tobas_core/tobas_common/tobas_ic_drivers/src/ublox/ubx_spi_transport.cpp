// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_spi_transport.hpp"

#include <cstring>

namespace tobas
{
namespace ublox
{
UBXSPITransport::UBXSPITransport() : rate_(kReqInterval)
{
}

bool UBXSPITransport::initialize(const char* device)
{
  return spi_.initialize(device, tx_buf_, rx_buf_, kSpiClockFreq);
}

void UBXSPITransport::startReceive()
{
  rate_.start();
}

bool UBXSPITransport::receiveByte(uint8_t& data)
{
  if (!spi_.transfer(1)) {
    return false;
  }

  data = rx_buf_[0];
  return true;
}

void UBXSPITransport::waitReceiveInterval()
{
  rate_.sleep();
}

bool UBXSPITransport::send(const uint8_t* data, size_t length)
{
  if (length > kSpiBufSize) {
    return false;
  }

  std::memcpy(tx_buf_, data, length);
  return spi_.transfer(length);
}
}  // namespace ublox
}  // namespace tobas
