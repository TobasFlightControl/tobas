// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace cyclonedds
{
struct SharedMemory
{
  bool enable = false;

  enum LogLevel
  {
    kVerbose,
    kDebug,
    kInfo,
    kWarn,
    kError,
    kFatal,
    kOff,
  } log_level = kInfo;
};
}  // namespace cyclonedds
}  // namespace tobas
