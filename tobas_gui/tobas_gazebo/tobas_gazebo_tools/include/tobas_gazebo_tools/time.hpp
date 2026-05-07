// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/msgs/time.pb.h>

namespace tobas
{
namespace gazebo
{
inline int64_t nanoseconds(const gz::msgs::Time& stamp)
{
  return stamp.sec() * 1'000'000'000L + static_cast<int64_t>(stamp.nsec());
}

inline double microseconds(const gz::msgs::Time& stamp)
{
  return static_cast<double>(nanoseconds(stamp)) * 1e-3;
}

inline double milliseconds(const gz::msgs::Time& stamp)
{
  return static_cast<double>(nanoseconds(stamp)) * 1e-6;
}

inline double seconds(const gz::msgs::Time& stamp)
{
  return static_cast<double>(nanoseconds(stamp)) * 1e-9;
}
}  // namespace gazebo
}  // namespace tobas
